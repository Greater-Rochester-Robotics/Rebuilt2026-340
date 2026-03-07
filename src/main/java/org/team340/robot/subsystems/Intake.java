package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's intake.
 */
@Logged
public final class Intake extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("intake");

    private static final TunableDouble pivotAcceleration = tunables.value("pivotAcceleration", 22.0);
    private static final TunableDouble pivotStowed = tunables.value("pivotStowed", 0.0); // TODO

    private static enum State {
        STOW(-0.08, 6.0, 0.0),
        PURGE(-0.08, 6.0, -90.0),
        EXTEND(0.262, 9.0, 0.0),
        INTAKE(0.262, 9.0, 90.0),
        COMPRESS(0.05, 2.0, 90.0),
        BARF(0.262, 9.0, -90.0);

        public final TunableDouble position;
        public final TunableDouble pivotVelocity;
        public final TunableDouble rollerVelocity;

        private State(final double position, final double pivotVelocity, final double rollerVelocity) {
            this.position = tunables.value("positions/" + name(), position);
            this.pivotVelocity = tunables.value("pivotVelocity/" + name(), pivotVelocity);
            this.rollerVelocity = tunables.value("rollerVelocities/" + name(), rollerVelocity);
        }
    }

    private final TalonFX pivot;
    private final TalonFX rollers;
    private final CANcoder wcpThroughborePoweredByCANcoderForHalfInchHex; // do not change this name

    private final StatusSignal<Angle> pivotPosition;

    private final DynamicMotionMagicVoltage pivotPositionControl;
    private final VelocityTorqueCurrentFOC rollersVelocityControl;

    public Intake() {
        this.pivot = new TalonFX(RobotMap.INTAKE_PIVOT_MOTOR, RobotMap.CANBus);
        this.rollers = new TalonFX(RobotMap.INTAKE_ROLLER_MOTOR, RobotMap.CANBus);
        this.wcpThroughborePoweredByCANcoderForHalfInchHex = new CANcoder(
            RobotMap.INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX,
            RobotMap.CANBus
        );

        configureCANcoder();
        configurePivot();
        configureRollers();

        pivotPosition = pivot.getPosition();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(100, pivotPosition, pivot.getClosedLoopReference())
        );
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(4, pivot, rollers, wcpThroughborePoweredByCANcoderForHalfInchHex)
        );

        pivotPositionControl = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
        pivotPositionControl.EnableFOC = true;
        pivotPositionControl.UpdateFreqHz = 0.0;

        rollersVelocityControl = new VelocityTorqueCurrentFOC(0.0);
        rollersVelocityControl.UpdateFreqHz = 0.0;

        tunables.add("pivotMotor", pivot);
        tunables.add("rollersMotor", rollers);

        // Enum warmup
        State.STOW.position.get();
    }

    @Override
    public void periodic() {
        pivotPosition.refresh();
    }

    /**
     * Stows the intake.
     */
    public Command stow() {
        return runState(State.STOW).withName("Intake.stow()");
    }

    /**
     * intake stows and barfs at the same time
     */
    public Command purge() {
        return runState(State.PURGE).withName("Intake.purge");
    }

    /**
     * Extends the intake without moving the rollers.
     */
    public Command extend() {
        return runState(State.EXTEND).withName("Intake.extend()");
    }

    /**
     * Extends the intake and runs the rollers to pick up fuel.
     */
    public Command intake() {
        return runState(State.INTAKE).withName("Intake.intake()");
    }

    /**
     * Extends the intake and runs the rollers, when {@code runRollers} returns {@code true}.
     * @param runRollers A boolean supplier that returns {@code true} when the rollers should be powered.
     */
    public Command intake(BooleanSupplier runRollers) {
        return sequence(intake().until(() -> !runRollers.getAsBoolean()), extend().until(runRollers))
            .repeatedly()
            .withName("Intake.intake()");
    }

    /**
     * Retracts the intake and runs the rollers inwards. Meant to be
     * used for compressing balls into the indexing system.
     */
    public Command compress() {
        return runState(State.COMPRESS).withName("Intake.compress()");
    }

    /**
     * Extends the intake and runs the rollers to barf out fuel.
     */
    public Command barf() {
        return runState(State.BARF).withName("Intake.barf()");
    }

    /**
     * Internal method to run the motors as configured for the specified state.
     * @param state The intake state to target.
     */
    private Command runState(State state) {
        return commandBuilder("Intake.runState()")
            .onExecute(() -> {
                pivotPositionControl.withPosition(state.position.getAsDouble());
                pivotPositionControl.withVelocity(state.pivotVelocity.get());
                pivotPositionControl.withAcceleration(pivotAcceleration.get());
                pivot.setControl(pivotPositionControl);

                rollersVelocityControl.withVelocity(state.rollerVelocity.getAsDouble());
                if (Math.abs(rollersVelocityControl.Velocity) > 1e-6) {
                    rollers.setControl(rollersVelocityControl);
                } else {
                    rollers.stopMotor();
                }
            })
            .onEnd(() -> {
                pivot.stopMotor();
                rollers.stopMotor();
            });
    }

    public boolean isStowed() {
        return pivotPosition.getValueAsDouble() <= pivotStowed.get();
    }

    private void configureCANcoder() {
        final CANcoderConfiguration wcpThroughborePoweredByCANcoderForHalfInchHexConfig = new CANcoderConfiguration();

        wcpThroughborePoweredByCANcoderForHalfInchHexConfig.MagnetSensor.MagnetOffset = 0.536;
        wcpThroughborePoweredByCANcoderForHalfInchHexConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive;

        PhoenixUtil.run(() ->
            wcpThroughborePoweredByCANcoderForHalfInchHex
                .getConfigurator()
                .apply(wcpThroughborePoweredByCANcoderForHalfInchHexConfig)
        );
    }

    private void configurePivot() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.ClosedLoopGeneral.ContinuousWrap = true;

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 35.0;

        config.Feedback.FeedbackRemoteSensorID = RobotMap.INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = (20.0 / 3.0) * 6.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 75.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 2.0;
        config.Slot0.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.281;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.150;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> pivot.clearStickyFaults());
        PhoenixUtil.run(() -> pivot.getConfigurator().apply(config));
    }

    private void configureRollers() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 20.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 3.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> rollers.clearStickyFaults());
        PhoenixUtil.run(() -> rollers.getConfigurator().apply(config));
    }
}
