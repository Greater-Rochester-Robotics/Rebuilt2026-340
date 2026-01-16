package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.SwerveEncoders.SwerveEncoder.HookStatus;
import org.team340.lib.swerve.hardware.SwerveMotors.SwerveMotor;

/**
 * An encapsulation of all hardware for a swerve module.
 */
class SwerveModule implements AutoCloseable {

    public final SwerveMotor moveMotor;
    public final SwerveMotor turnMotor;
    public final SwerveEncoder encoder;

    private final SwerveConfig config;
    private final SwerveModuleConfig moduleConfig;
    private final HookStatus hookStatus;

    private final SwerveModulePosition position = new SwerveModulePosition();
    private final SwerveModuleState state = new SwerveModuleState();
    private final SwerveModuleState lastTarget = new SwerveModuleState();
    private final SwerveModuleState nextTarget = new SwerveModuleState();

    private Lock cacheMutex = new ReentrantLock();
    private Rotation2d cachedAngle = Rotation2d.kZero;
    private double cachedTurnPosition = 0.0;

    /**
     * Creates the swerve module.
     * @param config The general swerve API configuration.
     * @param moduleConfig The module's configuration.
     */
    public SwerveModule(SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.config = config;
        this.moduleConfig = moduleConfig;

        moveMotor = SwerveMotor.construct(moduleConfig.moveMotor, config, true);
        turnMotor = SwerveMotor.construct(moduleConfig.turnMotor, config, false);
        encoder = SwerveEncoder.construct(moduleConfig.encoder, config, turnMotor);

        hookStatus = encoder.hookStatus();
    }

    /**
     * Gets the module's configured name.
     */
    public String getName() {
        return moduleConfig.name;
    }

    /**
     * Returns all Phoenix status signals in use by the module.
     */
    public List<BaseStatusSignal> getSignals() {
        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.addAll(moveMotor.getSignals());
        signals.addAll(turnMotor.getSignals());
        signals.addAll(encoder.getSignals());
        return signals;
    }

    /**
     * Refreshes the cached position and state of the module.
     * @return {@code true} on success, {@code false} if a read error occurred.
     */
    public boolean refresh() {
        double turnPosition = turnMotor.getPosition();
        Rotation2d angle = Rotation2d.fromRotations(hookStatus.readMotor() ? turnPosition : encoder.getPosition());
        double moveRotationsPerMeter = config.moveGearRatio / (config.wheelDiameter * Math.PI);

        position.distanceMeters = moveMotor.getPosition() / moveRotationsPerMeter;
        position.angle = angle;

        state.speedMetersPerSecond = moveMotor.getVelocity() / moveRotationsPerMeter;
        state.angle = angle;

        cacheMutex.lock();
        try {
            cachedAngle = angle;
            cachedTurnPosition = turnPosition;
        } finally {
            cacheMutex.unlock();
        }

        return !moveMotor.readError() && !turnMotor.readError() && !encoder.readError();
    }

    /**
     * Gets the module's position. The returned {@link SwerveModulePosition}
     * object is final and can be cached, but is volatile, as it may be
     * asynchronously refreshed by the odometry thread.
     */
    public SwerveModulePosition getPosition() {
        return position;
    }

    /**
     * Gets the module's state. The returned {@link SwerveModuleState}
     * object is final and can be cached, but is volatile, as it may
     * be asynchronously refreshed by the odometry thread.
     */
    public SwerveModuleState getState() {
        return state;
    }

    /**
     * Gets the module's last target state. The returned {@link SwerveModuleState}
     * object is final and can be cached, and is <i>not</i> volatile.
     */
    public SwerveModuleState getLastTarget() {
        return lastTarget;
    }

    /**
     * Gets the module's next target state. The returned {@link SwerveModuleState}
     * object is final and can be cached, and is <i>not</i> volatile.
     */
    public SwerveModuleState getNextTarget() {
        return nextTarget;
    }

    /**
     * Sets the target state of the swerve module.
     * @param state The state to apply to the module.
     */
    public void applyState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < config.velDeadband) {
            state.speedMetersPerSecond = 0.0;
            state.angle = nextTarget.angle;
        }

        Rotation2d angleDelta;
        double turnPosition;
        cacheMutex.lock();
        try {
            angleDelta = state.angle.minus(cachedAngle);
            turnPosition = cachedTurnPosition;
        } finally {
            cacheMutex.unlock();
        }

        boolean flipped = false;
        if (Math.abs(angleDelta.getRadians()) > Math2.HALF_PI) {
            state.speedMetersPerSecond *= -1.0;
            state.angle = state.angle.rotateBy(Rotation2d.kPi);
            flipped = true;
        }

        moveMotor.setVelocity(state.speedMetersPerSecond * (config.moveGearRatio / (config.wheelDiameter * Math.PI)));

        if (hookStatus.applyAbsolute()) {
            turnMotor.setPosition(state.angle.getRotations());
        } else {
            double optimizedDelta =
                angleDelta.getRadians() - (flipped ? Math.copySign(Math.PI, angleDelta.getRadians()) : 0.0);
            turnMotor.setPosition(turnPosition + (Units.radiansToRotations(optimizedDelta) * config.turnGearRatio));
        }

        Math2.copyInto(nextTarget, lastTarget);
        Math2.copyInto(state, nextTarget);
    }

    /**
     * Drives the swerve module using open-loop voltage. Intended for characterization.
     * @param voltage The voltage to apply to the move motor.
     * @param angle The angle to apply to the turn motor.
     */
    public void applyVoltage(double voltage, Rotation2d angle) {
        double target;
        cacheMutex.lock();
        try {
            if (hookStatus.applyAbsolute()) {
                target = MathUtil.inputModulus(angle.getRotations(), -0.5, 0.5);
            } else {
                target = cachedTurnPosition + (angle.minus(cachedAngle).getRotations() * config.turnGearRatio);
            }
        } finally {
            cacheMutex.unlock();
        }

        moveMotor.setVoltage(voltage);
        turnMotor.setPosition(target);
    }

    @Override
    public void close() {
        try {
            moveMotor.close();
            turnMotor.close();
            encoder.close();
        } catch (Exception e) {}
    }
}
