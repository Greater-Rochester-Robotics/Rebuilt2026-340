package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's roller floor and uptake.
 */
@Logged
public final class Indexer extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("indexer");

    private static enum State {
        ACCELERATE(55.0, 0.0),
        FEED(55.0, 50.0),
        CLEAR(-10.0, -10.0),
        BARF(-50.0, -50.0);

        public final TunableDouble uptakeSpeed;
        public final TunableDouble floorSpeed;

        private State(final double uptakeSpeed, final double floorSpeed) {
            this.uptakeSpeed = tunables.value("uptakeSpeed/" + name(), uptakeSpeed);
            this.floorSpeed = tunables.value("floorSpeed/" + name(), floorSpeed);
        }
    }

    private final TalonFX portUpperLead;
    private final TalonFX portLowerFollow;
    private final TalonFX starboardUpperFollow;
    private final TalonFX starboardLowerLead;

    private final MotionMagicVelocityVoltage velocityControl;
    private final Follower portFollowControl;
    private final Follower starboardFollowControl;

    private boolean isFeeding = false;

    public Indexer() {
        this.portUpperLead = new TalonFX(RobotMap.INDEXER_PORT_UPPER_MOTOR, RobotMap.CANBus);
        this.portLowerFollow = new TalonFX(RobotMap.INDEXER_PORT_LOWER_MOTOR, RobotMap.CANBus);
        this.starboardUpperFollow = new TalonFX(RobotMap.INDEXER_STARBOARD_UPPER_MOTOR, RobotMap.CANBus);
        this.starboardLowerLead = new TalonFX(RobotMap.INDEXER_STARBOARD_LOWER_MOTOR, RobotMap.CANBus);

        configurePortMotors();
        configureStarboardMotors();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                portUpperLead.getMotorVoltage(),
                starboardLowerLead.getMotorVoltage()
            )
        );
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                portUpperLead.getVelocity(),
                portUpperLead.getClosedLoopReference(),
                starboardLowerLead.getVelocity(),
                starboardLowerLead.getClosedLoopReference()
            )
        );
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(
                4,
                portUpperLead,
                portLowerFollow,
                starboardUpperFollow,
                starboardLowerLead
            )
        );

        velocityControl = new MotionMagicVelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        portFollowControl = new Follower(portUpperLead.getDeviceID(), MotorAlignmentValue.Aligned);
        starboardFollowControl = new Follower(starboardLowerLead.getDeviceID(), MotorAlignmentValue.Aligned);

        tunables.add("portMotors", portUpperLead);
        tunables.add("portMotors", portLowerFollow);
        tunables.add("starboardMotors", starboardUpperFollow);
        tunables.add("starboardMotors", starboardLowerLead);

        // Enum warmup
        State.FEED.uptakeSpeed.get();
    }

    @Override
    public void periodic() {
        portLowerFollow.setControl(portFollowControl);
        starboardUpperFollow.setControl(starboardFollowControl);

        if (DriverStation.isDisabled()) {
            portUpperLead.stopMotor();
            portLowerFollow.stopMotor();
            starboardUpperFollow.stopMotor();
            starboardLowerLead.stopMotor();
        }
    }

    /**
     * Returns {@code true} if the indexer is feeding the shooter.
     */
    public boolean isFeeding() {
        return isFeeding;
    }

    /**
     * Accelerates the uptake to feeding speed.
     */
    public Command accelerate() {
        return runState(State.ACCELERATE).withName("Indexer.accelerate()");
    }

    /**
     * Feeds the shooter.
     */
    public Command feed() {
        return runState(State.FEED)
            .alongWith(Commands.runEnd(() -> isFeeding = true, () -> isFeeding = false))
            .withName("Indexer.feed()");
    }

    /**
     * Runs the indexer to clear the ball tunnel.
     */
    public Command clear() {
        return runState(State.CLEAR).withTimeout(0.5).andThen(idle()).withName("Indexer.clear()");
    }

    /**
     * Barfs back into the hopper.
     */
    public Command barf() {
        return runState(State.BARF).withName("Indexer.barf()");
    }

    /**
     * Internal method to run the motors as configured for the specified state.
     * @param state The indexer state to target.
     */
    private Command runState(final State state) {
        return commandBuilder("Indexer.runState(" + state.name() + ")")
            .onExecute(() -> {
                velocityControl.withVelocity(state.uptakeSpeed.get());
                portUpperLead.setControl(velocityControl);
                velocityControl.withVelocity(state.floorSpeed.get());
                starboardLowerLead.setControl(velocityControl);
            })
            .onEnd(() -> {
                portUpperLead.stopMotor();
                starboardLowerLead.stopMotor();
            });
    }

    private void configurePortMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 180.0;
        config.CurrentLimits.SupplyCurrentLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotionMagic.MotionMagicAcceleration = 800.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.35;
        config.Slot0.kV = 0.127;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> portUpperLead.clearStickyFaults());
        PhoenixUtil.run(() -> portUpperLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> portLowerFollow.clearStickyFaults());
        PhoenixUtil.run(() -> portLowerFollow.getConfigurator().apply(config));
    }

    private void configureStarboardMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 180.0;
        config.CurrentLimits.SupplyCurrentLimit = 65.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 3.0;

        config.MotionMagic.MotionMagicAcceleration = 800.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.39;
        config.Slot0.kV = 0.14;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> starboardUpperFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardUpperFollow.getConfigurator().apply(config));

        PhoenixUtil.run(() -> starboardLowerLead.clearStickyFaults());
        PhoenixUtil.run(() -> starboardLowerLead.getConfigurator().apply(config));
    }
}
