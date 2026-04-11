package org.team340.robot.subsystems;

import static org.team340.robot.util.ShootParams.hoodPositionMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's hood.
 */
@Logged
public final class Hood extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("hood");

    private static final TunableDouble atPositionEpsilon = tunables.value("atPositionEpsilon", 2.0);
    private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.05);
    private static final TunableDouble homingVelocity = tunables.value("homingVelocity", -30.0); // In rotations per second.
    private static final TunableDouble zeroZero = tunables.value("zeroZero", 1.0); // In rotations per second.

    private final TalonFX motor;

    private final StatusSignal<Double> closedLoopError;
    private final StatusSignal<AngularVelocity> velocity;

    private final PositionVoltage positionVoltage;
    private final VelocityTorqueCurrentFOC velocityTorque;

    private boolean isZeroed = false;

    public Hood() {
        this.motor = new TalonFX(RobotMap.HOOD_MOTOR, RobotMap.CANBus);

        configureMotor();

        closedLoopError = motor.getClosedLoopError();
        velocity = motor.getVelocity();

        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, closedLoopError, velocity));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getPosition(), motor.getClosedLoopReference())
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, motor));

        positionVoltage = new PositionVoltage(0.0);
        positionVoltage.EnableFOC = true;
        positionVoltage.UpdateFreqHz = 0.0;

        velocityTorque = new VelocityTorqueCurrentFOC(0.0);
        velocityTorque.IgnoreSoftwareLimits = true;
        velocityTorque.UpdateFreqHz = 0.0;
        velocityTorque.Slot = 1;

        tunables.add("motor", motor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(closedLoopError, velocity);
    }

    /**
     * Checks if the hood is at its position within {@link Hood#closedLoopError}.
     * @return True if the hood is at the position, false if it is not.
     */
    public boolean atPosition() {
        return Math.abs(closedLoopError.getValueAsDouble()) <= atPositionEpsilon.get();
    }

    /**
     * Run the hood pivot to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return goTo(() -> hoodPositionMap.get(distance.getAsDouble())).withName("Hood.targetDistance()");
    }

    /**
     * Moves the hood to zero.
     * @param reZero If the zeroing sequence should also be ran.
     */
    public Command goToZero(boolean reZero) {
        Command goTo = goTo(zeroZero).withName("Hood.goToZero(" + reZero + ")");
        if (reZero) goTo = goTo.beforeStarting(() -> isZeroed = false);
        return goTo;
    }

    /**
     * Internal method to target a specified position.
     * @param position The hood's position in rotations at the rotor (gearing not included).
     */
    private Command goTo(final DoubleSupplier position) {
        Debouncer debouncer = new Debouncer(0.1, DebounceType.kRising);

        return commandBuilder("Hood.goTo()")
            .onInitialize(() -> debouncer.calculate(false))
            .onExecute(() -> {
                if (!isZeroed) {
                    velocityTorque.withVelocity(homingVelocity.get());
                    motor.setControl(velocityTorque);
                    if (!debouncer.calculate(Math.abs(velocity.getValueAsDouble()) < stallVelocity.get())) return;

                    isZeroed = true;
                    motor.setPosition(0.0);
                }

                positionVoltage.withPosition(position.getAsDouble());
                PhoenixUtil.run(() -> motor.setControl(positionVoltage));
            })
            .onEnd(motor::stopMotor);
    }

    private void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Normal operations
        config.Slot0.kP = 16.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.08;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        // Zeroing the hood.
        config.Slot1.kP = 12.0;
        config.Slot1.kI = 0.0; // If this is anything other than zero, it should not be.
        config.Slot1.kD = 0.0;
        config.Slot1.kG = 0.0;
        config.Slot1.kS = 0.0;
        config.Slot1.kV = 0.0;
        config.Slot1.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 22.938;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

        PhoenixUtil.run(() -> motor.clearStickyFaults());
        PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
    }
}
