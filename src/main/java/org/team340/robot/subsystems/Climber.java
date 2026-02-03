package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Climber extends GRRSubsystem {

    private final TalonFX lead;
    private final TalonFX follow;

    private final PositionVoltage positionVoltage;

    public Climber() {
        this.lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);

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

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                lead.getDutyCycle(),
                lead.getMotorVoltage(),
                lead.getTorqueCurrent()
            )
        );

        positionVoltage = new PositionVoltage(0.0);
        positionVoltage.EnableFOC = true;
        positionVoltage.UpdateFreqHz = 0.0;

        final Follower followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.Aligned);
        PhoenixUtil.run(() -> follow.setControl(followControl));
    }

    /**
     * Run the climber to the specified position.
     * @param position The position in revolutions.
     */
    public Command goTo(final double position) {
        return commandBuilder("Climber.goTo(" + position + ")")
            .onExecute(() -> {
                positionVoltage.withPosition(position);
                lead.setControl(positionVoltage);
            })
            .onEnd(() -> {
                lead.stopMotor();
            });
    }
}
