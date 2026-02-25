package org.team340.robot.subsystems;

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
 * the robot's indexer and uptake.
 */
@Logged
public final class Indexer extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("indexer");

    private static enum State {
        FEED(83.0, 72.0),
        UNJAM(-80.0, -80.0);

        public final TunableDouble twindexerSpeed;
        public final TunableDouble updateSpeed;

        private State(final double twindexerSpeed, final double updateSpeed) {
            this.twindexerSpeed = tunables.value("twindexerSpeeds/" + name(), twindexerSpeed);
            this.updateSpeed = tunables.value("uptakeSpeeds/" + name(), updateSpeed);
        }
    }

    final TalonFX twindexer;
    final TalonFX uptake;

    private final VelocityVoltage velocityControl;

    public Indexer() {
        this.twindexer = new TalonFX(RobotMap.INDEXER_TWINDEXER_MOTOR, RobotMap.CANBus);
        this.uptake = new TalonFX(RobotMap.INDEXER_UPTAKE_MOTOR, RobotMap.CANBus);

        configureTwindexer();
        configureUptake();

        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, twindexer, uptake));

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        tunables.add("twindexerMotor", twindexer);
        tunables.add("uptakeMotor", uptake);
    }

    public Command feed() {
        return run(State.FEED).withName("Indexer.feed()");
    }

    public Command unjam() {
        return run(State.UNJAM).withName("Indexer.unjam()");
    }

    private Command run(final State state) {
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
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
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
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
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
