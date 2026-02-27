package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
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

    private static final TunableDouble climbingVoltage = tunables.value("climbingVoltage", -3.5);
    private static final TunableDouble zeroingVelocity = tunables.value("zeroingVelocity", 6.0);
    private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.05);
    private static final TunableDouble atPositionEpsilon = tunables.value("atPositionEpsilon", 0.2);

    private static enum Position {
        TOP(9.1),
        BOTTOM(-32.598),
        L3(-10.0),
        ZERO(0.0),
        RETRACTING(-5.25); // This is the position we go to before we retract.

        public TunableDouble value;

        private Position(final double value) {
            this.value = tunables.value("position/" + name(), value);
        }
    }

    private final TalonFX lead;
    private final TalonFX follow;
    private final CANcoder zeroSwitch;

    private boolean isZeroed = false;

    private final MotionMagicVoltage positionControl;
    private final VelocityTorqueCurrentFOC velocityControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    private final StatusSignal<Angle> position;
    private final StatusSignal<MagnetHealthValue> seesMagnet;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;

    public Climber() {
        this.lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.zeroSwitch = new CANcoder(RobotMap.CLIMBER_CANCODER, RobotMap.CANBus);

        this.position = lead.getPosition();
        this.seesMagnet = zeroSwitch.getMagnetHealth();
        this.leadVelocity = lead.getVelocity();
        this.followVelocity = follow.getVelocity();

        configureCANcoder();
        configureMotors();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                lead.getDutyCycle(),
                lead.getMotorVoltage(),
                lead.getTorqueCurrent(),
                zeroSwitch.getMagnetHealth()
            )
        );
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, position, leadVelocity, followVelocity));
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, lead, follow, zeroSwitch));

        positionControl = new MotionMagicVoltage(0.0);
        positionControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        positionControl.EnableFOC = true;
        positionControl.UpdateFreqHz = 0.0;

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.IgnoreSoftwareLimits = true; // Software limits are not reliable during zeroing
        velocityControl.UpdateFreqHz = 0.0;
        velocityControl.Slot = 1;

        voltageControl = new VoltageOut(0.0);
        voltageControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        voltageControl.EnableFOC = false;
        voltageControl.UpdateFreqHz = 0.0;

        followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.Aligned);
        followControl.UpdateFreqHz = 0.0;

        tunables.add("motor", lead);
        tunables.add("motor", follow);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(seesMagnet, leadVelocity, followVelocity);
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

    public Command climbL3(BooleanSupplier ready) {
        return sequence(
            goTo(Position.TOP, false),
            waitUntil(ready),
            goTo(Position.BOTTOM, true),
            waitSeconds(0.25),
            goTo(Position.TOP, false),
            waitSeconds(0.25),
            goTo(Position.BOTTOM, true),
            waitSeconds(0.25),
            goTo(Position.TOP, false),
            waitSeconds(0.25),
            goTo(Position.L3, true)
        );
    }

    /**
     * Zero the climber.
     */
    public Command zero() {
        Debouncer debouncer = new Debouncer(0.1, DebounceType.kRising);

        return commandBuilder("Climber.zero()")
            .onInitialize(() -> debouncer.calculate(false))
            .onExecute(() -> {
                velocityControl.withVelocity(zeroingVelocity.get());
                lead.setControl(velocityControl);
                follow.setControl(followControl);
            })
            .isFinished(() -> {
                if (atZero()) {
                    isZeroed = true;
                    return true;
                }

                if (
                    debouncer.calculate(
                        Math.abs(leadVelocity.getValueAsDouble()) < stallVelocity.get()
                            || Math.abs(followVelocity.getValueAsDouble()) < stallVelocity.get()
                    )
                ) {
                    // Set here to avoid rechecking (or having the stator current change concurrently).
                    PhoenixUtil.run(() -> lead.setPosition(Position.TOP.value.get()));
                    PhoenixUtil.run(() -> follow.setPosition(Position.TOP.value.get()));
                    isZeroed = true;
                    return true;
                }

                return false;
            })
            .onEnd(() -> {
                lead.stopMotor();
                follow.stopMotor();
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
                          voltageControl.withOutput(climbingVoltage.get());
                          lead.setControl(voltageControl);
                          follow.setControl(voltageControl);
                      }
                    : () -> {
                          positionControl.withPosition(position.value.get());
                          lead.setControl(positionControl);
                          follow.setControl(followControl);
                      }
            )
            .onEnd(() -> {
                lead.stopMotor();
                follow.stopMotor();
            })
            .isFinished(() -> {
                if (!loaded) return (
                    Math.abs(this.position.getValueAsDouble() - position.value.get()) < atPositionEpsilon.get()
                );
                else return this.position.getValueAsDouble() <= position.value.get() + atPositionEpsilon.get();
            });

        return sequence(zero().onlyIf(() -> !isZeroed), goTo).withName("Climber.goTo(" + position + ")");
    }

    private void configureCANcoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        PhoenixUtil.run(() -> zeroSwitch.clearStickyFaults());
        PhoenixUtil.run(() -> zeroSwitch.getConfigurator().apply(config));
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.CLIMBER_CANCODER;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 115.0;
        config.MotionMagic.MotionMagicAcceleration = 900.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Normal operations
        config.Slot0.kP = 12.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.15;
        config.Slot0.kA = 0.0;

        // Zeroing the climber.
        config.Slot1.kP = 12.0;
        config.Slot1.kI = 0.0; // If this is anything other than zero, it should not be.
        config.Slot1.kD = 0.0;
        config.Slot1.kG = 0.0;
        config.Slot1.kS = 0.0;
        config.Slot1.kV = 0.0;
        config.Slot1.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.1;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -32.598;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 15.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -15.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));
    }
}
