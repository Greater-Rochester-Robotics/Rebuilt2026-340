package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Intake extends GRRSubsystem {

    private enum States {
        DOWN(0.0, 0.0),
        UP(0.0, 0.0);

        public final TunableDouble harvesterPosition;
        public final TunableDouble rollerVelocity;

        private States(final double harvesterPosition, final double rollerVelocity) {
            this.harvesterPosition = Tunables.value("Intake/" + name() + "/harvesterPosition", harvesterPosition);
            this.rollerVelocity = Tunables.value("Intake/" + name() + "/rollerVelocity", rollerVelocity);
        }
    }

    private TalonFX pivot;
    private TalonFX rollers;
    private CANcoder wcpThroughborePoweredByCANcoderForHalfInchHex;

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
            ParentDevice.optimizeBusUtilizationForAll(50, pivot, rollers, wcpThroughborePoweredByCANcoderForHalfInchHex)
        );

        pivotPositionControl = new MotionMagicVoltage(0.0);
        pivotPositionControl.EnableFOC = true;
        pivotPositionControl.UpdateFreqHz = 0.0;

        rollersVelocityControl = new VelocityVoltage(0.0);
        rollersVelocityControl.EnableFOC = true;
        rollersVelocityControl.UpdateFreqHz = 0.0;
    }

    public Command intake(States state) {
        return commandBuilder("Intake.intake()")
            .onInitialize(() -> {
                pivotPositionControl.withPosition(state.harvesterPosition.getAsDouble());
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
        // This config restores factory defaults.
        final CANcoderConfiguration wcpThroughborePoweredByCANcoderForHalfInchHexConfig = new CANcoderConfiguration();

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

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> rollers.clearStickyFaults());
        PhoenixUtil.run(() -> rollers.getConfigurator().apply(config));
    }
}
