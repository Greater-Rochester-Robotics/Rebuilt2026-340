package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
        FEED(83.0, 72.0),
        BARF(-80.0, -80.0);

        public final TunableDouble twindexerSpeed;
        public final TunableDouble updateSpeed;

        private State(final double twindexerSpeed, final double updateSpeed) {
            this.twindexerSpeed = tunables.value("twindexerSpeeds/" + name(), twindexerSpeed);
            this.updateSpeed = tunables.value("uptakeSpeeds/" + name(), updateSpeed);
        }
    }

    private final TalonFX twindexer;
    private final TalonFX uptake;

    private final VelocityVoltage velocityControl;

    public Indexer() {
        this.twindexer = new TalonFX(RobotMap.INDEXER_TWINDEXER_MOTOR, RobotMap.CANBus);
        this.uptake = new TalonFX(RobotMap.INDEXER_UPTAKE_MOTOR, RobotMap.CANBus);

        configureTwindexer();
        configureUptake();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(50, twindexer.getVelocity(), uptake.getVelocity())
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, twindexer, uptake));

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        tunables.add("twindexerMotor", twindexer);
        tunables.add("uptakeMotor", uptake);
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
                velocityControl.withVelocity(state.updateSpeed.get());
                uptake.setControl(velocityControl);
            })
            .onEnd(() -> {
                twindexer.stopMotor();
                uptake.stopMotor();
            });
    }

    private void configureTwindexer() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.127;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> twindexer.clearStickyFaults());
        PhoenixUtil.run(() -> twindexer.getConfigurator().apply(config));
    }

    private void configureUptake() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.132;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> uptake.clearStickyFaults());
        PhoenixUtil.run(() -> uptake.getConfigurator().apply(config));
    }
}
