package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Climber extends GRRSubsystem {

    private final TalonFX lead;
    private final TalonFX follow;
    private final CANcoder zeroSwitch;

    private final MotionMagicVoltage positionControl;

    public Climber() {
        this.lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.zeroSwitch = new CANcoder(RobotMap.CLIMBER_CANCODER, RobotMap.CANBus);

        configureCANcoder();
        configureMotors();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                lead.getDutyCycle(),
                lead.getMotorVoltage(),
                lead.getTorqueCurrent()
            )
        );

        positionControl = new MotionMagicVoltage(0.0);
        positionControl.EnableFOC = true;
        positionControl.UpdateFreqHz = 0.0;

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
                positionControl.withPosition(position);
                lead.setControl(positionControl);
            })
            .onEnd(() -> {
                lead.stopMotor();
            });
    }

    private void configureCANcoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        PhoenixUtil.run(() -> zeroSwitch.clearStickyFaults());
        PhoenixUtil.run(() -> zeroSwitch.getConfigurator().apply(config));
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.CLIMBER_CANCODER;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));
    }
}
