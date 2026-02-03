package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Indexer extends GRRSubsystem {

    public enum States {
        INTAKE(0.0, 0.0);

        public final TunableDouble twindexerSpeed;
        public final TunableDouble updateSpeed;

        private States(final double twindexerSpeed, final double updateSpeed) {
            this.twindexerSpeed = Tunables.value("Indexer/" + name() + "/twindexerSpeed", twindexerSpeed);
            this.updateSpeed = Tunables.value("Indexer/" + name() + "/updateSpeed", updateSpeed);
        }
    }

    final TalonFX twindexer;
    final TalonFX uptake;

    private final VelocityVoltage velocityControl;

    public Indexer() {
        this.twindexer = new TalonFX(RobotMap.INDEXER_TWINDEXER_MOTOR);
        this.uptake = new TalonFX(RobotMap.INDEXER_UPTAKE_MOTOR);

        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> twindexer.clearStickyFaults());
        PhoenixUtil.run(() -> twindexer.getConfigurator().apply(config));

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> uptake.clearStickyFaults());
        PhoenixUtil.run(() -> uptake.getConfigurator().apply(config));

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                twindexer.getDutyCycle(),
                twindexer.getMotorVoltage(),
                twindexer.getTorqueCurrent(),
                uptake.getDutyCycle(),
                uptake.getMotorVoltage(),
                uptake.getTorqueCurrent()
            )
        );

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;
    }

    public Command run(final States state) {
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
}
