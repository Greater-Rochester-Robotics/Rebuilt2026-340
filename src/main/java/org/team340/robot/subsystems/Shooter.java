package org.team340.robot.subsystems;

import static org.team340.robot.util.ShootParams.getShooterVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's shooter.
 */
@Logged
public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");

    private static final TunableDouble atVelocityEpsilon = tunables.value("atVelocityEpsilon", 1.5);
    private static final TunableDouble tuneVelocity = tunables.value("tuneVelocity", 0.0);

    private final TalonFX portUpperFollow;
    private final TalonFX portLowerLead;
    private final TalonFX starboardUpperFollow;
    private final TalonFX starboardLowerFollow;

    private final StatusSignal<Double> closedLoopError;

    private final VelocityVoltage velocityControl;
    private final StrictFollower followControl;

    public Shooter() {
        this.portUpperFollow = new TalonFX(RobotMap.SHOOTER_PORT_UPPER_MOTOR, RobotMap.CANBus);
        this.portLowerLead = new TalonFX(RobotMap.SHOOTER_PORT_LOWER_MOTOR, RobotMap.CANBus);
        this.starboardUpperFollow = new TalonFX(RobotMap.SHOOTER_STARBOARD_UPPER_MOTOR, RobotMap.CANBus);
        this.starboardLowerFollow = new TalonFX(RobotMap.SHOOTER_STARBOARD_LOWER_MOTOR, RobotMap.CANBus);

        configureMotors();

        closedLoopError = portLowerLead.getClosedLoopError();

        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, closedLoopError));
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(500, portLowerLead.getMotorVoltage()));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                portLowerLead.getVelocity(),
                portLowerLead.getClosedLoopReference()
            )
        );
        PhoenixUtil.run(() ->
            ParentDevice.optimizeBusUtilizationForAll(
                4,
                portUpperFollow,
                portLowerLead,
                starboardUpperFollow,
                starboardLowerFollow
            )
        );

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        followControl = new StrictFollower(portLowerLead.getDeviceID());

        tunables.add("motors", portUpperFollow);
        tunables.add("motors", portLowerLead);
        tunables.add("motors", starboardUpperFollow);
        tunables.add("motors", starboardLowerFollow);
    }

    @Override
    public void periodic() {
        closedLoopError.refresh();

        portUpperFollow.setControl(followControl);
        starboardUpperFollow.setControl(followControl);
        starboardLowerFollow.setControl(followControl);

        if (DriverStation.isDisabled()) {
            portUpperFollow.stopMotor();
            portLowerLead.stopMotor();
            starboardUpperFollow.stopMotor();
            starboardLowerFollow.stopMotor();
        }
    }

    /**
     * shooter's closed loop error is less than allowed error.
     * @return true if less
     */
    public boolean atVelocity() {
        return Math.abs(closedLoopError.getValueAsDouble()) <= atVelocityEpsilon.get();
    }

    /**
     * Run the shooter to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance, final BooleanSupplier inOurZone) {
        return runVelocity(() -> getShooterVelocity(distance.getAsDouble(), inOurZone.getAsBoolean())).withName(
            "Shooter.targetDistance()"
        );
    }

    /**
     * Runs the shooter at {@link #tuneVelocity}, for tuning purposes.
     */
    public Command tune() {
        return runVelocity(tuneVelocity).withName("Shooter.tune()");
    }

    /**
     * Internal method to run the shooter at a specified velocity.
     * @param velocity The velocity in rotations/second at the rotor (gearing not included).
     */
    private Command runVelocity(final DoubleSupplier velocity) {
        return commandBuilder("Shooter.runVelocity()")
            .onExecute(() -> {
                velocityControl.withVelocity(velocity.getAsDouble());
                portLowerLead.setControl(velocityControl);
            })
            .onEnd(portLowerLead::stopMotor);
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.35;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.128;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> portUpperFollow.clearStickyFaults());
        PhoenixUtil.run(() -> portUpperFollow.getConfigurator().apply(config));

        PhoenixUtil.run(() -> portLowerLead.clearStickyFaults());
        PhoenixUtil.run(() -> portLowerLead.getConfigurator().apply(config));

        // Inverts the direction for the other side.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> starboardUpperFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardUpperFollow.getConfigurator().apply(config));

        PhoenixUtil.run(() -> starboardLowerFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardLowerFollow.getConfigurator().apply(config));
    }
}
