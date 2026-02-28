package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
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

    private static enum State {
        // TODO positions
        STOW(0.0, 0.0),
        EXTEND(0.0, 0.0),
        INTAKE(0.0, 90.0),
        BARF(0.0, -90.0);

        public final TunableDouble position;
        public final TunableDouble rollerVelocity;

        private State(final double position, final double rollerVelocity) {
            this.position = tunables.value("positions/" + name(), position);
            this.rollerVelocity = tunables.value("rollerVelocities/" + name(), rollerVelocity);
        }
    }

    private final TalonFX pivot;
    private final TalonFX rollers;
    private final CANcoder wcpThroughborePoweredByCANcoderForHalfInchHex; // do not change this name

    private final MotionMagicVoltage pivotPositionControl;
    private final VelocityVoltage rollersVelocityControl;

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

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(50, pivot.getPosition(), pivot.getClosedLoopReference())
        );
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(4, pivot, rollers, wcpThroughborePoweredByCANcoderForHalfInchHex)
        );

        pivotPositionControl = new MotionMagicVoltage(0.0);
        pivotPositionControl.EnableFOC = true;
        pivotPositionControl.UpdateFreqHz = 0.0;

        rollersVelocityControl = new VelocityVoltage(0.0);
        rollersVelocityControl.EnableFOC = true;
        rollersVelocityControl.UpdateFreqHz = 0.0;

        tunables.add("pivotMotor", pivot);
        tunables.add("rollersMotor", rollers);
    }

    /**
     * Stows the intake.
     */
    public Command stow() {
        return runState(State.STOW).withName("Intake.stow()");
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
                pivot.setControl(pivotPositionControl);
                rollersVelocityControl.withVelocity(state.rollerVelocity.getAsDouble());
                rollers.setControl(rollersVelocityControl);
            })
            .onEnd(() -> {
                pivot.stopMotor();
                rollers.stopMotor();
            });
    }

    private void configureCANcoder() {
        final CANcoderConfiguration wcpThroughborePoweredByCANcoderForHalfInchHexConfig = new CANcoderConfiguration();

        // TODO
        wcpThroughborePoweredByCANcoderForHalfInchHexConfig.MagnetSensor.MagnetOffset = 0.0;
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

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // TODO: Confirm the direction in testing.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> pivot.clearStickyFaults());
        PhoenixUtil.run(() -> pivot.getConfigurator().apply(config));
    }

    private void configureRollers() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.3;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.124;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> rollers.clearStickyFaults());
        PhoenixUtil.run(() -> rollers.getConfigurator().apply(config));
    }
}
