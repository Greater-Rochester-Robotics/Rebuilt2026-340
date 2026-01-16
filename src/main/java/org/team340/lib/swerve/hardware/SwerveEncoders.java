package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.function.BiFunction;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders.SwerveEncoder.HookStatus;
import org.team340.lib.swerve.hardware.SwerveMotors.SwerveMotor;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.lib.util.vendors.ReduxUtil;
import org.team340.lib.util.vendors.RevUtil;

/**
 * Contains implementations for absolute encoders to be used with the {@link SwerveAPI}.
 */
public final class SwerveEncoders {

    private SwerveEncoders() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * A swerve module's absolute encoder.
     * All units are in rotations.
     */
    public abstract static class SwerveEncoder implements SwerveBaseHardware {

        /**
         * {@code (config, turnMotor) -> SwerveEncoder}
         */
        @FunctionalInterface
        public static interface Ctor extends BiFunction<SwerveConfig, SwerveMotor, SwerveEncoder> {}

        /** {@link SwerveEncoder#hookStatus()} */
        public static final record HookStatus(boolean readMotor, boolean applyAbsolute) {}

        /**
         * Constructs a swerve encoder. Wraps to support simulation if applicable.
         * @param ctor The encoder's constructor.
         * @param config The general swerve API configuration.
         * @param turnMotor The turn motor associated with the encoder's module.
         */
        public static SwerveEncoder construct(Ctor ctor, SwerveConfig config, SwerveMotor turnMotor) {
            SwerveEncoder encoder = ctor.apply(config, turnMotor);
            if (RobotBase.isSimulation()) encoder = simulate(encoder, config, turnMotor);
            return encoder;
        }

        /**
         * Gets the encoder's position in rotations.
         */
        public abstract double getPosition();

        /**
         * Some motor controllers can be configured to use external encoders as
         * a feedback device for closed-loop control. This method returns a
         * {@link HookStatus} that specifies if the encoder's corresponding
         * turn motor has been configured to return an absolute position via
         * its {@code getPosition()} method ({@link HookStatus#readMotor}),
         * and/or accept an absolute position when setting its closed loop
         * position target ({@link HookStatus#applyAbsolute}).
         */
        public HookStatus hookStatus() {
            return new HookStatus(false, false);
        }
    }

    /**
     * Configures a Spark Max attached duty cycle absolute encoder.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor sparkMaxEncoder(double offset, boolean inverted) {
        return (config, turnMotor) -> {
            if (!(turnMotor.getAPI() instanceof SparkMax sparkMax)) {
                throw new UnsupportedOperationException("Turn motor is not a Spark Max");
            }

            SparkAbsoluteEncoder encoder = sparkMax.getAbsoluteEncoder();
            HookStatus hookStatus = new HookStatus(false, true);

            var sparkConfig = new SparkMaxConfig();

            sparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-0.5, 0.5);

            sparkConfig.absoluteEncoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0)
                .zeroCentered(true)
                .averageDepth(2)
                .inverted(inverted)
                .zeroOffset(offset);

            sparkConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (config.odometryPeriod * 1000.0));

            RevUtil.configNoReset(sparkMax, sparkConfig);

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return encoder.getPosition();
                }

                @Override
                public HookStatus hookStatus() {
                    return hookStatus;
                }

                @Override
                public Object getAPI() {
                    return encoder;
                }

                @Override
                public void close() {}
            };
        };
    }

    /**
     * Configures a Spark Flex attached duty cycle absolute encoder.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor sparkFlexEncoder(double offset, boolean inverted) {
        return (config, turnMotor) -> {
            if (!(turnMotor.getAPI() instanceof SparkFlex sparkFlex)) {
                throw new UnsupportedOperationException("Turn motor is not a Spark Flex");
            }

            SparkAbsoluteEncoder encoder = sparkFlex.getAbsoluteEncoder();
            HookStatus hookStatus = new HookStatus(false, true);

            var sparkConfig = new SparkFlexConfig();

            sparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-0.5, 0.5);

            sparkConfig.absoluteEncoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0)
                .zeroCentered(true)
                .averageDepth(2)
                .inverted(inverted)
                .zeroOffset(offset);

            sparkConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (config.odometryPeriod * 1000.0));

            RevUtil.configNoReset(sparkFlex, sparkConfig);

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return encoder.getPosition();
                }

                @Override
                public HookStatus hookStatus() {
                    return hookStatus;
                }

                @Override
                public Object getAPI() {
                    return encoder;
                }

                @Override
                public void close() {}
            };
        };
    }

    /**
     * Configures a {@link Canandmag}.
     * @param id CAN ID of the device, as configured in Alchemist.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor canandmag(int id, double offset, boolean inverted) {
        return (config, turnMotor) -> {
            Canandmag canandmag = new Canandmag(id);

            var settings = new CanandmagSettings()
                .setDisableZeroButton(true)
                .setInvertDirection(inverted)
                .setPositionFramePeriod(config.odometryPeriod)
                .setStatusFramePeriod(config.defaultFramePeriod)
                .setVelocityFramePeriod(config.odometryPeriod)
                .setZeroOffset(offset);

            canandmag.clearStickyFaults();
            ReduxUtil.applySettings(canandmag, settings);

            if (RobotBase.isSimulation()) {
                CanandEventLoop.getInstance().setDevicePresenceWarnings(canandmag, false);
            }

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return ReduxUtil.latencyCompensate(canandmag.getAbsPositionFrame(), canandmag.getVelocity());
                }

                @Override
                public Object getAPI() {
                    return canandmag;
                }

                @Override
                public void close() {
                    canandmag.close();
                }
            };
        };
    }

    /**
     * Configures a {@link CANcoder}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor cancoder(int id, double offset, boolean inverted) {
        return (config, turnMotor) -> {
            CANcoder cancoder = new CANcoder(id, config.phoenixCanBus);
            HookStatus tempHookStatus = new HookStatus(false, false);

            StatusSignal<Angle> position = cancoder.getPosition(false).clone();
            StatusSignal<AngularVelocity> velocity = cancoder.getVelocity(false).clone();

            var cancoderConfig = new CANcoderConfiguration();
            cancoderConfig.MagnetSensor.MagnetOffset = offset;
            cancoderConfig.MagnetSensor.SensorDirection = inverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

            PhoenixUtil.run(() -> cancoder.clearStickyFaults());
            PhoenixUtil.run(() -> cancoder.getConfigurator().apply(cancoderConfig));
            PhoenixUtil.run(() ->
                BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.odometryPeriod, position, velocity)
            );
            PhoenixUtil.run(() -> cancoder.optimizeBusUtilization(1.0 / config.defaultFramePeriod, 0.05));

            if (turnMotor.getAPI() instanceof TalonFX talonFX) {
                var feedbackConfig = new FeedbackConfigs();
                talonFX.getConfigurator().refresh(feedbackConfig);

                feedbackConfig.FeedbackRemoteSensorID = id;
                feedbackConfig.RotorToSensorRatio = config.turnGearRatio;
                feedbackConfig.FeedbackSensorSource = config.phoenixPro
                    ? FeedbackSensorSourceValue.FusedCANcoder
                    : FeedbackSensorSourceValue.RemoteCANcoder;

                var closedLoopConfig = new ClosedLoopGeneralConfigs();
                talonFX.getConfigurator().refresh(closedLoopConfig);
                closedLoopConfig.ContinuousWrap = true;

                PhoenixUtil.run(() -> talonFX.getConfigurator().apply(feedbackConfig));
                PhoenixUtil.run(() -> talonFX.getConfigurator().apply(closedLoopConfig));

                tempHookStatus = new HookStatus(true, true);
            }

            HookStatus hookStatus = tempHookStatus;

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
                }

                @Override
                public HookStatus hookStatus() {
                    return hookStatus;
                }

                @Override
                public Object getAPI() {
                    return cancoder;
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(position, velocity);
                }

                @Override
                public void close() {
                    cancoder.close();
                }
            };
        };
    }

    /**
     * Rudimentary encoder simulation wrapper. Follows the position of the turn motor.
     * @param encoder The encoder to wrap.
     * @param config The general swerve API configuration.
     * @param turnMotor The turn motor associated with the encoder's module.
     */
    private static SwerveEncoder simulate(SwerveEncoder encoder, SwerveConfig config, SwerveMotor turnMotor) {
        return new SwerveEncoder() {
            @Override
            public double getPosition() {
                return (turnMotor.getPosition() / (hookStatus().readMotor() ? 1.0 : config.turnGearRatio));
            }

            @Override
            public HookStatus hookStatus() {
                return encoder.hookStatus();
            }

            @Override
            public Object getAPI() {
                return encoder.getAPI();
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return encoder.getSignals();
            }

            @Override
            public boolean readError() {
                return encoder.readError();
            }

            @Override
            public void close() {
                try {
                    encoder.close();
                } catch (Exception e) {}
            }
        };
    }
}
