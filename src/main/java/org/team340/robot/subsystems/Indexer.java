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

    private static enum State {
        FEED(85.0, 78.0),
        BARF(-80.0, -75.0);

        public final TunableDouble twindexerSpeed;
        public final TunableDouble uptakeSpeed;

        private State(final double twindexerSpeed, final double uptakeSpeed) {
            this.twindexerSpeed = tunables.value("twindexerSpeeds/" + name(), twindexerSpeed);
            this.uptakeSpeed = tunables.value("uptakeSpeeds/" + name(), uptakeSpeed);
        }
    }

    private final TalonFX twindexer;
    private final TalonFX uptakeLead;
    private final TalonFX uptakeFollow;

    private final VelocityTorqueCurrentFOC velocityControl;
    private final Follower uptakeFollowControl;

    public Indexer() {
        this.twindexer = new TalonFX(RobotMap.INDEXER_TWINDEXER_MOTOR, RobotMap.CANBus);
        this.uptakeLead = new TalonFX(RobotMap.INDEXER_UPTAKE_LEAD_MOTOR, RobotMap.CANBus);
        this.uptakeFollow = new TalonFX(RobotMap.INDEXER_UPTAKE_FOLLOW_MOTOR, RobotMap.CANBus);

        configureTwindexer();
        configureUptake();

        PhoenixUtil.run(() -> uptakeLead.getTorqueCurrent().setUpdateFrequency(500));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(50, twindexer.getVelocity(), uptakeLead.getVelocity())
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, twindexer, uptakeLead, uptakeFollow));

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.UpdateFreqHz = 0.0;

        uptakeFollowControl = new Follower(RobotMap.INDEXER_UPTAKE_LEAD_MOTOR, MotorAlignmentValue.Opposed);

        tunables.add("twindexerMotor", twindexer);
        tunables.add("uptakeLeadMotor", uptakeLead);
        tunables.add("uptakeFollowMotor", uptakeFollow);

        // Enum warmup
        State.FEED.twindexerSpeed.get();
    }

    @Override
    public void periodic() {
        uptakeFollow.setControl(uptakeFollowControl);
    }

    /**
     * Feeds the shooters.
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

    /**
     * Internal method to run the motors as configured for the specified state.
     * @param state The indexer state to target.
     */
    private Command runState(final State state) {
        return commandBuilder("Indexer.run()")
            .onExecute(() -> {
                velocityControl.withVelocity(state.twindexerSpeed.get());
                twindexer.setControl(velocityControl);

                velocityControl.withVelocity(state.uptakeSpeed.get());
                uptakeLead.setControl(velocityControl);
            })
            .onEnd(() -> {
                twindexer.stopMotor();
                uptakeLead.stopMotor();
            });
    }

    private void configureTwindexer() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 150.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 16.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 6.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> twindexer.clearStickyFaults());
        PhoenixUtil.run(() -> twindexer.getConfigurator().apply(config));
    }

    private void configureUptake() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 170.0;
        config.CurrentLimits.SupplyCurrentLimit = 65.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 11.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 8.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> uptakeLead.clearStickyFaults());
        PhoenixUtil.run(() -> uptakeLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> uptakeFollow.clearStickyFaults());
        PhoenixUtil.run(() -> uptakeFollow.getConfigurator().apply(config));
    }
}
