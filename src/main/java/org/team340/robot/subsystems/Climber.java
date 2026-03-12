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
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.CommandBuilder;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's climber.
 */
@Logged
public final class Climber extends GRRSubsystem {

    private static final double SERVO_DEPLOYED = 0.0;
    private static final double SERVO_RETRACT = 1.0;

    private static final TunableTable tunables = Tunables.getNested("climber");

    private static final TunableDouble zeroingVelocity = tunables.value("zeroingVelocity", 6.0);
    private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.05);
    private static final TunableDouble stallPosition = tunables.value("stallPosition", 13.318);
    private static final TunableDouble atPositionEpsilon = tunables.value("atPositionEpsilon", 0.15);

    private static enum Position {
        TOP(13.15),
        BOTTOM(-52.5),
        ZERO(0.32),
        RETRACTING(-7.5), // This is the position we go to before we retract.
        L1(-11.0),
        L3(-28.0); // -47.0 ?

        public TunableDouble value;

        private Position(final double value) {
            this.value = tunables.value("position/" + name(), value);
        }
    }

    private final TalonFX lead;
    private final TalonFX follow;
    private final PWM servo;
    private final CANcoder zeroSwitch;

    private final StatusSignal<Angle> position;
    private final StatusSignal<MagnetHealthValue> seesMagnet;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;

    private final DynamicMotionMagicVoltage loadedPositionControl;
    private final DynamicMotionMagicVoltage unloadedPositionControl;
    private final VelocityTorqueCurrentFOC velocityControl;
    private final Follower followControl;

    private boolean isZeroed = false;
    private boolean retracted = false;
    private boolean climbingL1 = false;

    public Climber() {
        this.lead = new TalonFX(RobotMap.CLIMBER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.CLIMBER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.servo = new PWM(RobotMap.CLIMBER_SERVO_CHANNEL);
        this.zeroSwitch = new CANcoder(RobotMap.CLIMBER_CANCODER, RobotMap.CANBus);

        this.position = lead.getPosition();
        this.seesMagnet = zeroSwitch.getMagnetHealth();
        this.leadVelocity = lead.getVelocity();
        this.followVelocity = follow.getVelocity();

        configureCANcoder();
        configureMotors();

        servo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
        servo.setPeriodMultiplier(PeriodMultiplier.k4X);

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(500, lead.getMotorVoltage(), lead.getTorqueCurrent(), seesMagnet)
        );
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                leadVelocity,
                followVelocity,
                lead.getClosedLoopReference()
            )
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, lead, follow, zeroSwitch));

        loadedPositionControl = new DynamicMotionMagicVoltage(0.0, 75.0, 500.0);
        loadedPositionControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        loadedPositionControl.EnableFOC = true;
        loadedPositionControl.UpdateFreqHz = 0.0;
        loadedPositionControl.Slot = 0;

        unloadedPositionControl = new DynamicMotionMagicVoltage(0.0, 115.0, 900.0);
        unloadedPositionControl.IgnoreHardwareLimits = true; // Hardware limits are only used for zeroing.
        unloadedPositionControl.EnableFOC = true;
        unloadedPositionControl.UpdateFreqHz = 0.0;
        unloadedPositionControl.Slot = 1;

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.IgnoreSoftwareLimits = true; // Software limits are not reliable during zeroing
        velocityControl.UpdateFreqHz = 0.0;
        velocityControl.Slot = 2;

        followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.Aligned);

        tunables.add("motor", lead);
        tunables.add("motor", follow);

        // Enum warmup
        Position.TOP.value.get();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, seesMagnet, leadVelocity, followVelocity);
        follow.setControl(followControl);

        // Allows for zeroing while disabled
        if (DriverStation.isDisabled() && atZero()) isZeroed = true;
    }

    /**
     * {@code true} if the climber is on the hall effect sensor.
     */
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
     * {@code true} if the climber hooks are on L1.
     */
    public boolean isClimbingL1() {
        return climbingL1;
    }

    /**
     * Climb L1.
     * @param ready If we are ready to start the actual climbing sequence.
     */
    public Command climbL1(BooleanSupplier ready) {
        return sequence(
            goTo(Position.TOP, false, false, false).until(ready),
            runOnce(() -> climbingL1 = true),
            goTo(Position.L1, true, false, false)
        ).withName("Climber.climbL1()");
    }

    /**
     * Unclimb the robot from L1 if it had been climbing on L1.
     */
    public Command unclimbL1() {
        return goTo(Position.TOP, true, false)
            .andThen(() -> climbingL1 = false)
            .onlyIf(this::isClimbingL1)
            .withName("Climber.unclimbL1()");
    }

    /**
     * Climb L3.
     * @param ready If we are ready to start the actual climbing sequence.
     */
    public Command climbL3(BooleanSupplier ready) {
        return sequence(
            goTo(Position.TOP, false, true, false).until(ready),
            goTo(Position.BOTTOM, true, true),
            waitSeconds(0.1),
            goTo(Position.TOP, false, true),
            waitSeconds(0.1),
            goTo(Position.BOTTOM, true, true),
            waitSeconds(0.1),
            goTo(Position.TOP, false, true),
            waitSeconds(0.1),
            // goTo(Position.BOTTOM, true, true),
            // waitSeconds(0.1),
            goTo(Position.L3, true, true, false)
        ).withName("Climber.climbL3()");
    }

    /**
     * Zero the climber.
     */
    public Command zero() {
        Debouncer debouncer = new Debouncer(0.2, DebounceType.kRising);

        return commandBuilder("Climber.zero()")
            .onInitialize(() -> debouncer.calculate(false))
            .onExecute(() -> {
                servo.setPosition(SERVO_RETRACT);
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
                    PhoenixUtil.run(() -> lead.setPosition(stallPosition.get()));
                    PhoenixUtil.run(() -> follow.setPosition(stallPosition.get()));
                    isZeroed = true;
                    return true;
                }

                return false;
            })
            .onEnd(lead::stopMotor);
    }

    /**
     * Moves the hooks to be retracted into the robot. This command does not end.
     * @param safe If the climber is safe to go to the retracted position.
     */
    public Command retract(BooleanSupplier safe) {
        return sequence(
            waitUntil(safe),
            zero().onlyIf(() -> !isZeroed),
            goTo(Position.RETRACTING, false, false).onlyIf(
                () -> position.getValueAsDouble() > Position.RETRACTING.value.get()
            ),
            goTo(Position.ZERO, false, false)
        )
            .onlyIf(() -> !isZeroed || !retracted)
            .andThen(idle())
            .withName("Climber.retract()");
    }

    /**
     * Internal method to target a specified position.
     * This command ends once it is within {@link Climber#atPositionEpsilon} of the target.
     * @param position The climber's position in rotations at the rotor (gearing not included).
     * @param deployServo True if the servo should be deployed, false if it should be
     */
    private Command goTo(final Position position, final boolean loaded, final boolean deployServo) {
        return goTo(position, loaded, deployServo, true);
    }

    /**
     * Internal method to target a specified position.
     * @param position The climber's position in rotations at the rotor (gearing not included).
     * @param deployServo True if the servo should be deployed, false if it should be
     * @param shouldFinish True if this command should finish when it is within {@link Climber#atPositionEpsilon}
     * of the target position, false if this command should not end.
     */
    private Command goTo(
        final Position position,
        final boolean loaded,
        final boolean deployServo,
        final boolean shouldFinish
    ) {
        BooleanSupplier atPosition = () ->
            Math.abs(this.position.getValueAsDouble() - position.value.get()) < atPositionEpsilon.get();

        CommandBuilder goTo = commandBuilder()
            .onExecute(() -> {
                servo.setPosition(deployServo ? SERVO_DEPLOYED : SERVO_RETRACT);
                if (loaded) {
                    loadedPositionControl.withPosition(position.value.get());
                    lead.setControl(loadedPositionControl);
                } else {
                    unloadedPositionControl.withPosition(position.value.get());
                    lead.setControl(unloadedPositionControl);
                }
            })
            .onEnd(() -> {
                lead.stopMotor();
                retracted = position.equals(Position.ZERO) && atPosition.getAsBoolean();
            });

        if (shouldFinish) {
            goTo.isFinished(atPosition);
        }

        return sequence(zero().onlyIf(() -> !isZeroed), goTo).withName(
            "Climber.goTo(" + position.name() + ", " + loaded + ", " + shouldFinish + ")"
        );
    }

    /**
     * Deploys the servo, without moving the climber.
     */
    public Command testServo() {
        return runEnd(() -> servo.setPosition(SERVO_DEPLOYED), () -> servo.setPosition(SERVO_RETRACT));
    }

    private void configureCANcoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        PhoenixUtil.run(() -> zeroSwitch.clearStickyFaults());
        PhoenixUtil.run(() -> zeroSwitch.getConfigurator().apply(config));
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimit = 90.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = RobotMap.CLIMBER_CANCODER;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Loaded control.
        config.Slot0.kP = 16.0;
        config.Slot0.kV = 0.18;

        // Unloaded control.
        config.Slot1.kP = 12.0;
        config.Slot1.kV = 0.15;

        // Zeroing the climber.
        config.Slot2.kP = 12.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 13.318;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -53.862;
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
