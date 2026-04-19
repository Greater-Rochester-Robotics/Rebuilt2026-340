package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

    private static final TunableDouble pivotAcceleration = tunables.value("pivotAcceleration", 18.0);
    private static final TunableDouble pivotStowed = tunables.value("pivotStowed", -0.07);

    private static enum State {
        STOW(-0.08, 1.0, 0.0),
        EXTEND(0.253, 1.0, 0.0),
        INTAKE(0.253, 1.0, 90.0),
        AGITATE_UP(0.17, 2.0, 30.0),
        AGITATE_DOWN(0.253, 2.0, 30.0),
        BARF(0.253, 1.0, -90.0),
        PURGE(-0.08, 1.0, -90.0);

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
    private final TalonFX rollersLead;
    private final TalonFX rollersFollow;
    private final CANcoder wcpThroughborePoweredByCANcoderForHalfInchHex; // do not change this name

    private final StatusSignal<Angle> pivotPosition;

    private final DynamicMotionMagicVoltage pivotPositionControl;
    private final VelocityTorqueCurrentFOC rollersVelocityControl;
    private final Follower pivotFollowControl;

    public Intake() {
        this.pivot = new TalonFX(RobotMap.INTAKE_PIVOT_MOTOR, RobotMap.CANBus);
        this.rollersLead = new TalonFX(RobotMap.INTAKE_ROLLER_LEAD_MOTOR, RobotMap.CANBus);
        this.rollersFollow = new TalonFX(RobotMap.INTAKE_ROLLER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.wcpThroughborePoweredByCANcoderForHalfInchHex = new CANcoder(
            RobotMap.INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX,
            RobotMap.CANBus
        );

        configureCANcoder();
        configurePivot();
        configureRollers();

        pivotPosition = pivot.getPosition();

        PhoenixUtil.run(() -> rollersLead.getTorqueCurrent().setUpdateFrequency(500));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(100, pivotPosition, pivot.getClosedLoopReference())
        );
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(
                4,
                pivot,
                rollersLead,
                rollersFollow,
                wcpThroughborePoweredByCANcoderForHalfInchHex
            )
        );

        pivotPositionControl = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
        pivotPositionControl.EnableFOC = true;
        pivotPositionControl.UpdateFreqHz = 0.0;

        rollersVelocityControl = new VelocityTorqueCurrentFOC(0.0);
        rollersVelocityControl.UpdateFreqHz = 0.0;

        pivotFollowControl = new Follower(RobotMap.INTAKE_ROLLER_LEAD_MOTOR, MotorAlignmentValue.Aligned);

        tunables.add("pivotMotor", pivot);
        tunables.add("rollersLeadMotor", rollersLead);
        tunables.add("rollersFollowMotor", rollersFollow);

        // Enum warmup
        State.STOW.position.get();
    }

    @Override
    public void periodic() {
        pivotPosition.refresh();
        rollersFollow.setControl(pivotFollowControl);

        if (DriverStation.isDisabled()) {
            pivot.stopMotor();
            rollersLead.stopMotor();
            rollersFollow.stopMotor();
        }
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
     * Agitates the hopper by jostling the intake up and down while pulling balls inwards.
     */
    public Command agitate() {
        return sequence(runState(State.AGITATE_UP).withTimeout(0.4), runState(State.AGITATE_DOWN).withTimeout(0.4))
            .repeatedly()
            .withName("Intake.agitate()");
    }

    /**
     * Extends the intake and runs the rollers to barf out fuel.
     */
    public Command barf() {
        return runState(State.BARF).withName("Intake.barf()");
    }

    /**
     * Retracts the intake and barfs out fuel.
     */
    public Command purge() {
        return runState(State.PURGE).withName("Intake.purge()");
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
                    rollersLead.setControl(rollersVelocityControl);
                } else {
                    rollersLead.stopMotor();
                }
            })
            .onEnd(() -> {
                pivot.stopMotor();
                rollersLead.stopMotor();
            });
    }

    /**
     * Returns {@code true} when the intake is stowed inside the frame perimeter.
     */
    public boolean isStowed() {
        return pivotPosition.getValueAsDouble() <= pivotStowed.get();
    }

    private void configureCANcoder() {
        final CANcoderConfiguration wcpThroughborePoweredByCANcoderForHalfInchHexConfig = new CANcoderConfiguration();

        wcpThroughborePoweredByCANcoderForHalfInchHexConfig.MagnetSensor.MagnetOffset = -0.309;
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

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        config.Feedback.FeedbackRemoteSensorID = RobotMap.INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = (20.0 / 3.0) * 6.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 90.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 2.0;
        config.Slot0.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.281;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.082;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> pivot.clearStickyFaults());
        PhoenixUtil.run(() -> pivot.getConfigurator().apply(config));
    }

    private void configureRollers() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 180.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
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

        PhoenixUtil.run(() -> rollersLead.clearStickyFaults());
        PhoenixUtil.run(() -> rollersLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> rollersFollow.clearStickyFaults());
        PhoenixUtil.run(() -> rollersFollow.getConfigurator().apply(config));
    }
}
