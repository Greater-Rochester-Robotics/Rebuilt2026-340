package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's twindexer and uptake.
 */
@Logged
public final class Indexer extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("indexer");

    private static final TunableDouble preDelay = Tunables.value("preDelay", 3.0);
    private static final TunableDouble loadTime = Tunables.value("loadTime", 5.0);
    private static final TunableDouble backoffInTime = Tunables.value("backoffInTime", 1.0);

    private static enum State {
        FEED(78.0),
        BARF(-75.0),
        PRELOAD(0.0),
        BACKOFF(0.0);

        public final TunableDouble speed;

        private State(final double speed) {
            this.speed = tunables.value("speed/" + name(), speed);
        }
    }

    private final TalonFX portUpperLead;
    private final TalonFX portLowerFollow;
    private final TalonFX starboardUpperFollow;
    private final TalonFX starboardLowerFollow;

    private final VelocityTorqueCurrentFOC velocityControl;
    private final Follower followControl;

    public Indexer() {
        this.portUpperLead = new TalonFX(RobotMap.INDEXER_PORT_UPPER_LEAD_MOTOR, RobotMap.CANBus);
        this.portLowerFollow = new TalonFX(RobotMap.INDEXER_PORT_LOWER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.starboardUpperFollow = new TalonFX(RobotMap.INDEXER_STARBOARD_UPPER_FOLLOW_MOTOR, RobotMap.CANBus);
        this.starboardLowerFollow = new TalonFX(RobotMap.INDEXER_STARBOARD_LOWER_FOLLOW_MOTOR, RobotMap.CANBus);

        configureMotors();

        PhoenixUtil.run(() -> portUpperLead.getTorqueCurrent().setUpdateFrequency(500));
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, portUpperLead.getVelocity()));
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(
                4,
                portUpperLead,
                portLowerFollow,
                starboardUpperFollow,
                starboardLowerFollow
            )
        );

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.UpdateFreqHz = 0.0;

        followControl = new Follower(portUpperLead.getDeviceID(), MotorAlignmentValue.Aligned);

        tunables.add("portLeadMotor", portUpperLead);

        // Enum warmup
        State.FEED.speed.get();
    }

    @Override
    public void periodic() {
        portLowerFollow.setControl(followControl);
        starboardUpperFollow.setControl(followControl);
        starboardLowerFollow.setControl(followControl);
    }

    /**
     * Feeds the shooter.
     */
    public Command feed() {
        return runState(State.FEED).withName("Indexer.feed()");
    }

    /**
     * Barfs back into the hopper.
     */
    public Command barf() {
        return runState(State.BARF).withName("Indexer.unjam()");
    }

    public Command preload() {
        return Commands.waitSeconds(preDelay.get())
            .andThen(runState(State.PRELOAD).withTimeout(loadTime.get()))
            .andThen(runState(State.BACKOFF).withTimeout(backoffInTime.get()));
    }

    /**
     * Internal method to run the motors as configured for the specified state.
     * @param state The indexer state to target.
     */
    private Command runState(final State state) {
        return commandBuilder("Indexer.run()")
            .onExecute(() -> {
                velocityControl.withVelocity(state.speed.get());
                portUpperLead.setControl(velocityControl);
            })
            .onEnd(() -> {
                portUpperLead.stopMotor();
            });
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 170.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 11.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 8.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> portUpperLead.clearStickyFaults());
        PhoenixUtil.run(() -> portUpperLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> portLowerFollow.clearStickyFaults());
        PhoenixUtil.run(() -> portLowerFollow.getConfigurator().apply(config));

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> starboardUpperFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardUpperFollow.getConfigurator().apply(config));

        PhoenixUtil.run(() -> starboardLowerFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardLowerFollow.getConfigurator().apply(config));
    }
}
