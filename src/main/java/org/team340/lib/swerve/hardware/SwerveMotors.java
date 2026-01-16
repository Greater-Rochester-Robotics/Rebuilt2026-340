package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.function.BiFunction;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.lib.util.vendors.RevUtil;

/**
 * Contains implementations for motors to be used with the {@link SwerveAPI}.
 */
public final class SwerveMotors {

    private SwerveMotors() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * A swerve motor.
     * All units are in rotations.
     */
    public abstract static class SwerveMotor implements SwerveBaseHardware {

        /**
         * {@code (config, isMoveMotor) -> SwerveMotor}
         */
        @FunctionalInterface
        public static interface Ctor extends BiFunction<SwerveConfig, Boolean, SwerveMotor> {}

        /**
         * Constructs a swerve motor. Wraps to support simulation if applicable.
         * @param ctor The motor's constructor.
         * @param config The general swerve API configuration.
         * @param isMoveMotor {@code true} if the motor is a move motor.
         */
        public static SwerveMotor construct(Ctor ctor, SwerveConfig config, boolean isMoveMotor) {
            SwerveMotor motor = ctor.apply(config, isMoveMotor);
            if (RobotBase.isSimulation()) motor = simulate(motor, config);
            return motor;
        }

        /**
         * Gets the motor's position in rotations.
         */
        public abstract double getPosition();

        /**
         * Sets the motor's closed-loop position target.
         * @param position The target position in rotations.
         */
        public abstract void setPosition(double position);

        /**
         * Gets the motor's velocity in rotations/second.
         */
        public abstract double getVelocity();

        /**
         * Sets the motor's closed-loop velocity target.
         * @param velocity The target velocity in rotations/second.
         */
        public abstract void setVelocity(double velocity);

        /**
         * Sets the motor's output voltage.
         * @param voltage The voltage to apply.
         */
        public abstract void setVoltage(double voltage);

        /**
         * Re-applies PID and FF gains from the swerve config. Used
         * for setting new gains after the config has been mutated.
         * <b>This is likely a blocking operation.</b>
         */
        public abstract void reapplyGains();

        /**
         * Configures the motor's brake mode from the swerve config.
         * Used for setting a new value after the config has been mutated.
         * <b>This is likely a blocking operation.</b>
         */
        public abstract void reapplyBrakeMode();
    }

    /**
     * Configures a {@link CANSparkMax Spark Max}.
     * @param id CAN ID of the device, as configured in the REV Hardware Client.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor sparkMax(int id, boolean inverted) {
        return (config, isMoveMotor) -> {
            SparkMax sparkMax = new SparkMax(id, MotorType.kBrushless);
            RelativeEncoder relativeEncoder = sparkMax.getEncoder();
            SparkClosedLoopController pid = sparkMax.getClosedLoopController();

            double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
            double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

            var sparkConfig = new SparkMaxConfig();

            sparkConfig
                .voltageCompensation(config.voltage)
                .smartCurrentLimit((int) (isMoveMotor ? config.moveStatorLimit : config.turnStatorLimit))
                .idleMode(
                    (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode) ? IdleMode.kBrake : IdleMode.kCoast
                )
                .inverted(inverted);

            sparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(pidGains[0], pidGains[1], pidGains[2])
                .feedForward.kS(ffGains[0])
                .kV(ffGains[1]);

            sparkConfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0)
                .uvwMeasurementPeriod(isMoveMotor ? 16 : 32)
                .uvwAverageDepth(isMoveMotor ? 2 : 8);

            sparkConfig.signals
                .appliedOutputPeriodMs((int) (config.defaultFramePeriod * 1000.0))
                .faultsAlwaysOn(true)
                .faultsPeriodMs((int) (config.defaultFramePeriod * 1000.0))
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (config.odometryPeriod * 1000.0))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs((int) (config.odometryPeriod * 1000.0));

            RevUtil.config(sparkMax, sparkConfig);
            sparkMax.setControlFramePeriodMs(0);

            if (isMoveMotor) RevUtil.run("Zero Encoder", sparkMax, () -> relativeEncoder.setPosition(0.0));

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return relativeEncoder.getPosition();
                }

                @Override
                public void setPosition(double position) {
                    pid.setSetpoint(position, ControlType.kPosition);
                }

                @Override
                public double getVelocity() {
                    return relativeEncoder.getVelocity();
                }

                @Override
                public void setVelocity(double velocity) {
                    pid.setSetpoint(velocity, ControlType.kVelocity);
                }

                @Override
                public void setVoltage(double voltage) {
                    sparkMax.setVoltage(voltage);
                }

                @Override
                public void reapplyGains() {
                    double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
                    double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

                    var newConfig = new SparkMaxConfig();
                    newConfig.closedLoop
                        .pid(pidGains[0], pidGains[1], pidGains[2])
                        .feedForward.kS(ffGains[0])
                        .kV(ffGains[1]);

                    RevUtil.configEphemeral(sparkMax, newConfig);
                }

                @Override
                public void reapplyBrakeMode() {
                    RevUtil.configEphemeral(
                        sparkMax,
                        new SparkMaxConfig().idleMode(
                            (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode)
                                ? IdleMode.kBrake
                                : IdleMode.kCoast
                        )
                    );
                }

                @Override
                public Object getAPI() {
                    return sparkMax;
                }

                @Override
                public boolean readError() {
                    return !sparkMax.getLastError().equals(REVLibError.kOk);
                }

                @Override
                public void close() {
                    sparkMax.close();
                }
            };
        };
    }

    /**
     * Configures a {@link CANSparkFlex Spark Flex}.
     * @param id CAN ID of the device, as configured in the REV Hardware Client.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor sparkFlex(int id, boolean inverted) {
        return (config, isMoveMotor) -> {
            SparkFlex sparkFlex = new SparkFlex(id, MotorType.kBrushless);
            RelativeEncoder relativeEncoder = sparkFlex.getEncoder();
            SparkClosedLoopController pid = sparkFlex.getClosedLoopController();

            double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
            double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

            var sparkConfig = new SparkFlexConfig();

            sparkConfig
                .voltageCompensation(config.voltage)
                .smartCurrentLimit((int) (isMoveMotor ? config.moveStatorLimit : config.turnStatorLimit))
                .idleMode(
                    (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode) ? IdleMode.kBrake : IdleMode.kCoast
                )
                .inverted(inverted);

            sparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(pidGains[0], pidGains[1], pidGains[2])
                .feedForward.kS(ffGains[0])
                .kV(ffGains[1]);

            sparkConfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0)
                .quadratureMeasurementPeriod(isMoveMotor ? 32 : 100)
                .quadratureAverageDepth(isMoveMotor ? 8 : 64);

            sparkConfig.signals
                .appliedOutputPeriodMs((int) (config.defaultFramePeriod * 1000.0))
                .faultsAlwaysOn(true)
                .faultsPeriodMs((int) (config.defaultFramePeriod * 1000.0))
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (config.odometryPeriod * 1000.0))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs((int) (config.odometryPeriod * 1000.0));

            RevUtil.config(sparkFlex, sparkConfig);
            sparkFlex.setControlFramePeriodMs(0);

            if (isMoveMotor) RevUtil.run("Zero Encoder", sparkFlex, () -> relativeEncoder.setPosition(0.0));

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return relativeEncoder.getPosition();
                }

                @Override
                public void setPosition(double position) {
                    pid.setSetpoint(position, ControlType.kPosition);
                }

                @Override
                public double getVelocity() {
                    return relativeEncoder.getVelocity();
                }

                @Override
                public void setVelocity(double velocity) {
                    pid.setSetpoint(velocity, ControlType.kVelocity);
                }

                @Override
                public void setVoltage(double voltage) {
                    sparkFlex.setVoltage(voltage);
                }

                @Override
                public void reapplyGains() {
                    double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
                    double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

                    var newConfig = new SparkFlexConfig();
                    newConfig.closedLoop
                        .pid(pidGains[0], pidGains[1], pidGains[2])
                        .feedForward.kS(ffGains[0])
                        .kV(ffGains[1]);

                    RevUtil.configEphemeral(sparkFlex, newConfig);
                }

                @Override
                public void reapplyBrakeMode() {
                    RevUtil.configEphemeral(
                        sparkFlex,
                        new SparkFlexConfig().idleMode(
                            (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode)
                                ? IdleMode.kBrake
                                : IdleMode.kCoast
                        )
                    );
                }

                @Override
                public Object getAPI() {
                    return sparkFlex;
                }

                @Override
                public boolean readError() {
                    return !sparkFlex.getLastError().equals(REVLibError.kOk);
                }

                @Override
                public void close() {
                    sparkFlex.close();
                }
            };
        };
    }

    /**
     * Configures a {@link TalonFX}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor talonFX(int id, boolean inverted) {
        return (config, isMoveMotor) -> {
            TalonFX talonFX = new TalonFX(id, config.phoenixCanBus);

            StatusSignal<Angle> position = talonFX.getPosition(false).clone();
            StatusSignal<AngularVelocity> velocity = talonFX.getVelocity(false).clone();

            boolean enableFOC = isMoveMotor ? config.phoenixMoveFOC : config.phoenixTurnFOC;

            PositionVoltage positionControl = new PositionVoltage(0.0);
            positionControl.EnableFOC = enableFOC;
            positionControl.UpdateFreqHz = 0.0;

            VelocityVoltage velocityControl = new VelocityVoltage(0.0);
            velocityControl.EnableFOC = enableFOC;
            velocityControl.UpdateFreqHz = 0.0;

            VoltageOut voltageControl = new VoltageOut(0.0);

            double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
            double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

            double statorLimit = isMoveMotor ? config.moveStatorLimit : config.turnStatorLimit;
            double supplyLimit = isMoveMotor ? config.moveSupplyLimit : config.turnSupplyLimit;

            var talonConfig = new TalonFXConfiguration();

            talonConfig.Audio.AllowMusicDurDisable = true;

            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = statorLimit;
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -statorLimit;

            talonConfig.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
            talonConfig.MotorOutput.NeutralMode = (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode)
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

            talonConfig.Slot0.kP = pidGains[0];
            talonConfig.Slot0.kI = pidGains[1];
            talonConfig.Slot0.kD = pidGains[2];
            talonConfig.Slot0.kS = ffGains[0];
            talonConfig.Slot0.kV = ffGains[1];

            PhoenixUtil.run(() -> talonFX.clearStickyFaults());
            PhoenixUtil.run(() -> talonFX.getConfigurator().apply(talonConfig));
            PhoenixUtil.run(() ->
                BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.odometryPeriod, position, velocity)
            );
            PhoenixUtil.run(() -> talonFX.optimizeBusUtilization(1.0 / config.defaultFramePeriod, 0.05));

            if (isMoveMotor) PhoenixUtil.run(() -> talonFX.setPosition(0.0));

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
                }

                @Override
                public void setPosition(double position) {
                    talonFX.setControl(positionControl.withPosition(position));
                }

                @Override
                public double getVelocity() {
                    return velocity.getValueAsDouble();
                }

                @Override
                public void setVelocity(double velocity) {
                    talonFX.setControl(velocityControl.withVelocity(velocity));
                }

                @Override
                public void setVoltage(double voltage) {
                    talonFX.setControl(voltageControl.withOutput(voltage));
                }

                @Override
                public void reapplyGains() {
                    double[] pidGains = isMoveMotor ? config.movePID : config.turnPID;
                    double[] ffGains = isMoveMotor ? config.moveFF : new double[] { 0.0, 0.0 };

                    var slot0Config = talonConfig.Slot0;
                    slot0Config.kP = pidGains[0];
                    slot0Config.kI = pidGains[1];
                    slot0Config.kD = pidGains[2];
                    slot0Config.kS = ffGains[0];
                    slot0Config.kV = ffGains[1];
                    talonFX.getConfigurator().apply(slot0Config);
                }

                @Override
                public void reapplyBrakeMode() {
                    var motorOutputConfig = talonConfig.MotorOutput;
                    motorOutputConfig.NeutralMode = (isMoveMotor ? config.moveBrakeMode : config.turnBrakeMode)
                        ? NeutralModeValue.Brake
                        : NeutralModeValue.Coast;

                    talonFX.getConfigurator().apply(motorOutputConfig);
                }

                @Override
                public Object getAPI() {
                    return talonFX;
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(position, velocity);
                }

                @Override
                public void close() {
                    talonFX.close();
                }
            };
        };
    }

    /**
     * Rudimentary motor simulation wrapper. Motor physics are not simulated,
     * rather the simulation assumes the motor perfectly tracks its setpoint.
     * @param motor The motor to wrap.
     * @param config The general swerve API configuration.
     */
    private static SwerveMotor simulate(SwerveMotor motor, SwerveConfig config) {
        return new SwerveMotor() {
            private double position = 0.0;
            private double velocity = 0.0;

            @Override
            public double getPosition() {
                return position;
            }

            @Override
            public void setPosition(double position) {
                velocity = (position - this.position) / config.period;
                this.position = position;
                motor.setPosition(position);
            }

            @Override
            public double getVelocity() {
                return velocity;
            }

            @Override
            public void setVelocity(double velocity) {
                position = position + (velocity * config.period);
                this.velocity = velocity;
                motor.setVelocity(velocity);
            }

            @Override
            public void setVoltage(double voltage) {
                // No-op in sim
                // Intended for characterization with a real robot
                velocity = 0.0;
                motor.setVoltage(voltage);
            }

            @Override
            public void reapplyGains() {
                motor.reapplyGains();
            }

            @Override
            public void reapplyBrakeMode() {
                motor.reapplyBrakeMode();
            }

            @Override
            public Object getAPI() {
                return motor.getAPI();
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return motor.getSignals();
            }

            @Override
            public boolean readError() {
                return motor.readError();
            }

            @Override
            public void close() {
                try {
                    motor.close();
                } catch (Exception e) {}
            }
        };
    }
}
