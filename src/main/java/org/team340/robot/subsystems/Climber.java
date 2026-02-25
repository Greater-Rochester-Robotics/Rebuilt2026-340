package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * the robot's climber.
 */
@Logged
public final class Climber extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("climber");

    private static final TunableDouble zeroingVelocity = tunables.value("zeroingVelocity", 0.0);
    private static final TunableDouble stallCurrent = tunables.value("stallCurrent", 0.0);
    private static final TunableDouble atPositionEpsilon = tunables.value("atPositionEpsilon", 0.0);

    private static enum Position {
        TOP(0.0),
        ZERO(0.0),
        RETRACTING(0.0); // This is the position we go to before we retract.

        public TunableDouble value;

        private Position(final double value) {
            this.value = tunables.value("position/" + name(), value);
        }
    }

    private final TalonFX lead;
    private final TalonFX follow;
    private final CANcoder zeroSwitch;

    private boolean isZeroed = false;

    private final MotionMagicVoltage unloadedPositionControl;
    private final DynamicMotionMagicVoltage loadedPositionControl;
    private final VelocityTorqueCurrentFOC velocityControl;

    private final StatusSignal<Angle> position;
    private final StatusSignal<MagnetHealthValue> seesMagnet;

    public Climber() {
        this.lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.zeroSwitch = new CANcoder(RobotMap.CLIMBER_CANCODER, RobotMap.CANBus);

        this.position = lead.getPosition();
        this.seesMagnet = zeroSwitch.getMagnetHealth();

        configureCANcoder();
        configureMotors();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                lead.getMotorVoltage(),
                lead.getTorqueCurrent(),
                zeroSwitch.getMagnetHealth()
            )
        );
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                lead.getPosition(),
                lead.getStatorCurrent(),
                follow.getStatorCurrent()
            )
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, lead, follow, zeroSwitch));

        unloadedPositionControl = new MotionMagicVoltage(0.0);
        unloadedPositionControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        unloadedPositionControl.EnableFOC = true;
        unloadedPositionControl.UpdateFreqHz = 0.0;

        loadedPositionControl = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
        loadedPositionControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        loadedPositionControl.EnableFOC = true;
        loadedPositionControl.UpdateFreqHz = 0.0;

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.UpdateFreqHz = 0.0;
        velocityControl.Slot = 1;

        final Follower followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.Aligned);
        PhoenixUtil.run(() -> follow.setControl(followControl));
    }

    @Override
    public void periodic() {
        seesMagnet.refresh();
    }

    public boolean atZero() {
        // Probably unnecessary, but also fancy looking switch.
        switch (seesMagnet.getValue()) {
            case Magnet_Green, Magnet_Orange:
                return true;
            case Magnet_Invalid, Magnet_Red:
            default:
                return false;
        }
    }

    /**
     * run climber to zero position
     * then zeroes encoder
     * @return
     */
    public Command zero() {
        return commandBuilder("Climber.zero()")
            .onExecute(() -> {
                velocityControl.withVelocity(zeroingVelocity.get());
                lead.setControl(velocityControl);
            })
            .isFinished(() -> {
                if (atZero()) {
                    isZeroed = true;
                    return true;
                }

                if (
                    lead.getStatorCurrent().getValueAsDouble() >= stallCurrent.get()
                    || follow.getStatorCurrent().getValueAsDouble() >= stallCurrent.get()
                ) {
                    // Set here to avoid rechecking (or having the stator current change concurrently).
                    PhoenixUtil.run(() -> lead.setPosition(Position.TOP.value.get()));
                    isZeroed = true;
                    return true;
                }

                return false;
            });
    }

    public Command retract() {
        return sequence(
            either(zero(), none(), () -> !isZeroed),
            either(
                goTo(Position.RETRACTING, false),
                none(),
                () -> position.getValueAsDouble() > Position.RETRACTING.value.get()
            ),
            goTo(Position.ZERO, false)
        );
    }

    /**
     * Run the climber to the specified Position.
     * Run zero() if isZeroed false
     * @param position The climber Position.
     */
    private Command goTo(final Position position, final boolean loaded) {
        Command goTo = commandBuilder()
            .onExecute(
                loaded
                    ? () -> {
                          loadedPositionControl.withPosition(position.value.get());
                          lead.setControl(loadedPositionControl);
                      }
                    : () -> {
                          unloadedPositionControl.withPosition(position.value.get());
                          lead.setControl(unloadedPositionControl);
                      }
            )
            .onEnd(() -> {
                lead.stopMotor();
            })
            .isFinished(
                () -> Math.abs(this.position.getValueAsDouble() - position.value.get()) < atPositionEpsilon.get()
            );
        return either(goTo, zero().andThen(goTo), () -> isZeroed).withName("Climber.goTo(" + position + ")");
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
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Normal operations
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        // Zeroing the climber.
        config.Slot1.kP = 0.0;
        config.Slot1.kI = 0.0; // If this is anything other than zero, it should not be.
        config.Slot1.kD = 0.0;
        config.Slot1.kG = 0.0;
        config.Slot1.kS = 0.0;
        config.Slot1.kV = 0.0;
        config.Slot1.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));
    }
}
