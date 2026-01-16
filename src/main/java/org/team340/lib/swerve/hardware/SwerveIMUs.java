package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveIMUs.SwerveIMU.IMUSimHook;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.lib.util.vendors.ReduxUtil;

/**
 * Contains implementations for IMUs to be used with the {@link SwerveAPI}.
 */
public final class SwerveIMUs {

    private SwerveIMUs() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * A swerve IMU.
     */
    public abstract static class SwerveIMU implements SwerveBaseHardware {

        /**
         * {@code (config) -> SwerveIMU}
         */
        @FunctionalInterface
        public static interface Ctor extends Function<SwerveConfig, SwerveIMU> {}

        /**
         * Provides consumer to be saved and invoked periodically with the
         * robot's current chassis speeds to update the simulated IMU.
         */
        @FunctionalInterface
        public static interface IMUSimHook extends Consumer<Consumer<ChassisSpeeds>> {}

        /**
         * Constructs a swerve IMU. Wraps to support simulation if applicable.
         * @param ctor The IMU's constructor.
         * @param config The general swerve API configuration.
         * @param simHook Hook to update the IMU if simulation is active.
         */
        public static SwerveIMU construct(Ctor ctor, SwerveConfig config, IMUSimHook simHook) {
            SwerveIMU imu = ctor.apply(config);
            if (RobotBase.isSimulation()) imu = simulate(imu, config, simHook);
            return imu;
        }

        /**
         * Gets the IMU's absolute yaw.
         */
        public abstract Rotation2d getYaw();

        /**
         * Gets the IMU's absolute pitch.
         */
        public abstract Rotation2d getPitch();

        /**
         * Gets the IMU's absolute roll.
         */
        public abstract Rotation2d getRoll();
    }

    /**
     * Configures an {@link ADIS16470_IMU ADIS16470 IMU}.
     * @param yawAxis The axis to use for yaw.
     * @param pitchAxis The axis to use for pitch.
     * @param rollAxis The axis to use for roll.
     * @param port The SPI port used.
     * @param calibrationTime The time frame to calibrate for.
     */
    public static SwerveIMU.Ctor adis16470(
        IMUAxis yawAxis,
        IMUAxis pitchAxis,
        IMUAxis rollAxis,
        SPI.Port port,
        CalibrationTime calibrationTime
    ) {
        return config -> {
            ADIS16470_IMU adis16470 = new ADIS16470_IMU(yawAxis, pitchAxis, rollAxis, port, calibrationTime);

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getYawAxis()));
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getPitchAxis()));
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getRollAxis()));
                }

                @Override
                public Object getAPI() {
                    return adis16470;
                }

                @Override
                public void close() {
                    adis16470.close();
                }
            };
        };
    }

    /**
     * Configures a {@link Canandgyro}.
     * @param id CAN ID of the device, as configured in Alchemist.
     */
    public static SwerveIMU.Ctor canandgyro(int id) {
        return config -> {
            Canandgyro canandgyro = new Canandgyro(id);

            var settings = new CanandgyroSettings()
                .setAccelerationFramePeriod(config.defaultFramePeriod)
                .setAngularPositionFramePeriod(config.odometryPeriod)
                .setAngularVelocityFramePeriod(config.odometryPeriod)
                .setStatusFramePeriod(config.defaultFramePeriod)
                .setYawFramePeriod(config.odometryPeriod);

            canandgyro.clearStickyFaults();
            ReduxUtil.applySettings(canandgyro, settings);

            if (RobotBase.isSimulation()) {
                CanandEventLoop.getInstance().setDevicePresenceWarnings(canandgyro, false);
            }

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromRotations(
                        ReduxUtil.latencyCompensate(canandgyro.getYawFrame(), canandgyro.getAngularVelocityYaw())
                    );
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromRotations(
                        ReduxUtil.latencyCompensateQuaternionPitch(
                            canandgyro.getAngularPositionFrame(),
                            canandgyro.getAngularVelocityPitch()
                        )
                    );
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromRotations(
                        ReduxUtil.latencyCompensateQuaternionRoll(
                            canandgyro.getAngularPositionFrame(),
                            canandgyro.getAngularVelocityRoll()
                        )
                    );
                }

                @Override
                public Object getAPI() {
                    return canandgyro;
                }

                @Override
                public void close() {
                    canandgyro.close();
                }
            };
        };
    }

    /**
     * Configures a {@link Pigeon2}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     */
    public static SwerveIMU.Ctor pigeon2(int id) {
        return config -> {
            Pigeon2 pigeon2 = new Pigeon2(id, config.phoenixCanBus);

            StatusSignal<Angle> yaw = pigeon2.getYaw(false).clone();
            StatusSignal<Angle> pitch = pigeon2.getPitch(false);
            StatusSignal<Angle> roll = pigeon2.getRoll(false);
            StatusSignal<AngularVelocity> yawVelocity = pigeon2.getAngularVelocityZWorld().clone();
            StatusSignal<AngularVelocity> pitchVelocity = pigeon2.getAngularVelocityXWorld();
            StatusSignal<AngularVelocity> rollVelocity = pigeon2.getAngularVelocityYWorld();

            PhoenixUtil.run(() -> pigeon2.clearStickyFaults());
            PhoenixUtil.run(() ->
                BaseStatusSignal.setUpdateFrequencyForAll(
                    1.0 / config.odometryPeriod,
                    yaw,
                    pitch,
                    roll,
                    yawVelocity,
                    pitchVelocity,
                    rollVelocity
                )
            );
            PhoenixUtil.run(() -> pigeon2.optimizeBusUtilization(1.0 / config.defaultFramePeriod, 0.05));

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity)
                    );
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(pitch.refresh(), pitchVelocity.refresh())
                    );
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValueAsDouble(roll.refresh(), rollVelocity.refresh())
                    );
                }

                @Override
                public Object getAPI() {
                    return pigeon2;
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(yaw, yawVelocity);
                }

                @Override
                public void close() {
                    pigeon2.close();
                }
            };
        };
    }

    /**
     * Rudimentary IMU simulation wrapper. Calculates yaw based on the robot's angular velocity.
     * @param imu The IMU to wrap.
     * @param config The general swerve API configuration.
     * @param simHook Hook to update the IMU if simulation is active.
     */
    private static SwerveIMU simulate(SwerveIMU imu, SwerveConfig config, IMUSimHook simHook) {
        Mutable<Double> yaw = new Mutable<>(0.0);
        simHook.accept(speeds -> {
            yaw.value += speeds.omegaRadiansPerSecond * config.period;
        });

        return new SwerveIMU() {
            @Override
            public Rotation2d getYaw() {
                return Rotation2d.fromRadians(yaw.value);
            }

            @Override
            public Rotation2d getPitch() {
                return Rotation2d.kZero;
            }

            @Override
            public Rotation2d getRoll() {
                return Rotation2d.kZero;
            }

            @Override
            public Object getAPI() {
                return imu.getAPI();
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return imu.getSignals();
            }

            @Override
            public boolean readError() {
                return imu.readError();
            }

            @Override
            public void close() {
                try {
                    imu.close();
                } catch (Exception e) {}
            }
        };
    }
}
