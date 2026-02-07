package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public final class Shooters extends GRRSubsystem {

    private static final TunableDouble manualAcceleration = Tunables.value("Shooters/manualAcceleration", 0.0);
    private static final InterpolatingDoubleTreeMap distanceVelocityMap;

    static {
        distanceVelocityMap = new InterpolatingDoubleTreeMap();

        // TODO: Populate data points.
        distanceVelocityMap.put(0.0, 0.0);
    }

    private final TalonFX portLead;
    private final TalonFX portFollow;
    private final TalonFX starboardLead;
    private final TalonFX starboardFollow;

    private final VelocityVoltage velocityControl;

    public Shooters() {
        this.portLead = new TalonFX(RobotMap.SHOOTER_PORT_LEAD_MOTOR, RobotMap.CANBus);
        this.portFollow = new TalonFX(RobotMap.SHOOTER_PORT_FOLLOW_MOTOR, RobotMap.CANBus);
        this.starboardLead = new TalonFX(RobotMap.SHOOTER_STARBOARD_LEAD_MOTOR, RobotMap.CANBus);
        this.starboardFollow = new TalonFX(RobotMap.SHOOTER_STARBOARD_FOLLOW_MOTOR, RobotMap.CANBus);

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

        PhoenixUtil.run(() -> portLead.clearStickyFaults());
        PhoenixUtil.run(() -> portLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> portFollow.clearStickyFaults());
        PhoenixUtil.run(() -> portFollow.getConfigurator().apply(config));

        // Inverts the direction for the other side.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> starboardLead.clearStickyFaults());
        PhoenixUtil.run(() -> starboardLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> starboardFollow.clearStickyFaults());
        PhoenixUtil.run(() -> starboardFollow.getConfigurator().apply(config));

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                portLead.getDutyCycle(),
                portLead.getMotorVoltage(),
                portLead.getTorqueCurrent(),
                starboardLead.getDutyCycle(),
                starboardLead.getMotorVoltage(),
                starboardLead.getTorqueCurrent()
            )
        );

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        final Follower portFollowControl = new Follower(portLead.getDeviceID(), MotorAlignmentValue.Aligned);
        PhoenixUtil.run(() -> portFollow.setControl(portFollowControl));

        final Follower starboardFollowControl = new Follower(starboardLead.getDeviceID(), MotorAlignmentValue.Aligned);
        PhoenixUtil.run(() -> starboardFollow.setControl(starboardFollowControl));
    }

    /**
     * Run the shooter to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return runVelocity(() -> distanceVelocityMap.get(distance.getAsDouble())).withName("Shooters.targetDistance()");
    }

    /**
     * Manually run the shooters faster or slower using a acceleration input.
     * @param acceleration A acceleration between [-1.0, 1.0] to drive the hood up or down. This is scaled by {@link Shooters#manualSpeed}.
     */
    public Command manualControl(final DoubleSupplier acceleration) {
        final Mutable<Double> velocity = new Mutable<>(0.0);

        return runVelocity(() -> velocity.value += -acceleration.getAsDouble() * manualAcceleration.get())
            .beforeStarting(() -> velocity.value = 0.0)
            .withName("Shooters.manualControl()");
    }

    /**
     * Internal method to run both shooters at a velocity.
     * @param velocity The velocity in rotations per second at the rotor (gearing not included).
     */
    private Command runVelocity(final DoubleSupplier velocity) {
        return commandBuilder("Shooter.runVelocity()")
            .onExecute(() -> {
                velocityControl.withVelocity(velocity.getAsDouble());
                portLead.setControl(velocityControl);
                starboardLead.setControl(velocityControl);
            })
            .onEnd(() -> {
                portLead.stopMotor();
                starboardLead.stopMotor();
            });
    }
}
