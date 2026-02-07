package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Hood extends GRRSubsystem {

    private static final TunableDouble manualSpeed = Tunables.value("Hood/manualSpeed", 0.0);
    private static final double homingVelocity = 0.0; // In rotations per second.
    private static final InterpolatingDoubleTreeMap distancePositionMap;

    static {
        distancePositionMap = new InterpolatingDoubleTreeMap();

        // TODO: Populate data points.
        distancePositionMap.put(0.0, 0.0);
    }

    private final TalonFX motor;
    private final CANdi zeroSwitch;

    private boolean isZeroed = false;

    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;

    private final StatusSignal<Boolean> zeroSwitchS1Closed;

    public Hood() {
        this.motor = new TalonFX(RobotMap.HOOD_MOTOR);
        this.zeroSwitch = new CANdi(RobotMap.HOOD_ZERO_SWITCH);

        this.zeroSwitchS1Closed = zeroSwitch.getS1Closed();

        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.HOOD_ZERO_SWITCH;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // This config restores factory defaults.
        final CANdiConfiguration candiConfig = new CANdiConfiguration();

        // TODO: Find out the direction of the motor.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> motor.clearStickyFaults());
        PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
        PhoenixUtil.run(() -> zeroSwitch.getConfigurator().apply(candiConfig));

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(500, zeroSwitch.getS1State(), zeroSwitch.getS1Closed())
        );

        positionVoltage = new PositionVoltage(0.0);
        positionVoltage.EnableFOC = true;
        positionVoltage.UpdateFreqHz = 0.0;

        velocityVoltage = new VelocityVoltage(0.0);
        velocityVoltage.EnableFOC = true;
        velocityVoltage.UpdateFreqHz = 0.0;
    }

    @Override
    public void periodic() {
        zeroSwitchS1Closed.refresh();
    }

    public boolean atZero() {
        return zeroSwitchS1Closed.getValue();
    }

    /**
     * Run the hood pivot to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return goTo(() -> distancePositionMap.get(distance.getAsDouble())).withName("Hood.targetDistance()");
    }

    /**
     * Manually run the hood up or down using a velocity input.
     * @param velocity A velocity between [-1.0, 1.0] to drive the hood up or down. This is scaled by {@link Hood#manualSpeed}.
     */
    public Command manualControl(final DoubleSupplier velocity) {
        final Mutable<Double> position = new Mutable<>(0.0);

        return goTo(() -> position.value += -velocity.getAsDouble() * manualSpeed.get())
            .beforeStarting(() -> position.value = motor.getPosition().getValueAsDouble())
            .withName("Hood.manualControl()");
    }

    /**
     * Runs the hood to a position.
     * @param position The position in revolutions.
     */
    private Command goTo(final DoubleSupplier position) {
        return commandBuilder("Hood.goTo()")
            .onExecute(() -> {
                if (!isZeroed) {
                    velocityVoltage.withVelocity(homingVelocity);
                    motor.setControl(velocityVoltage);
                    if (!atZero()) return;

                    isZeroed = true;
                }

                positionVoltage.withPosition(position.getAsDouble());
                motor.setControl(positionVoltage);
            })
            .onEnd(motor::stopMotor);
    }
}
