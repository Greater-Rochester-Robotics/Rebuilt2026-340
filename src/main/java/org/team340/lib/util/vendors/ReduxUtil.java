package org.team340.lib.util.vendors;

import com.reduxrobotics.frames.DoubleFrame;
import com.reduxrobotics.frames.Frame;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import com.reduxrobotics.sensors.canandgyro.QuaternionFrame;
import com.reduxrobotics.sensors.canandgyro.Vec3Frame;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;

/**
 * Utilities for ReduxLib.
 */
public final class ReduxUtil {

    private ReduxUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Applies settings to a Canandmag and checks for errors.
     * Will try up to 3 times if settings fail to apply.
     * @param canandmag The Canandmag.
     * @param settings The settings to apply.
     * @return {@code true} if success, {@code false} otherwise.
     */
    public static boolean applySettings(Canandmag canandmag, CanandmagSettings settings) {
        return applySettings(canandmag, settings, 3);
    }

    /**
     * Applies settings to a Canandmag and checks for errors.
     * @param canandmag The Canandmag.
     * @param settings The settings to apply.
     * @param maxTries The number of times to try to apply settings before failing. {@code 1} only tries once.
     * @return {@code true} if success, {@code false} otherwise.
     */
    public static boolean applySettings(Canandmag canandmag, CanandmagSettings settings, int maxTries) {
        if (RobotBase.isSimulation()) return true;

        String results = "";
        for (int i = 0; i < maxTries; i++) {
            if (!canandmag.setSettings(settings, 0.05)) {
                results += (results.isEmpty() ? "" : ", ") + "setSettings() Failure";
                continue;
            }

            boolean failed = false;
            Map<Integer, Long> device = canandmag.getSettings(0.5).getFilteredMap();
            for (var entry : settings.getFilteredMap().entrySet()) {
                if (device.get(entry.getKey()) != entry.getValue()) {
                    results += (results.isEmpty() ? "" : ", ") + "Failed check";
                    failed = true;
                    break;
                }
            }

            if (!failed) return true;
        }

        DriverStation.reportError(
            "[ReduxUtil] Canandmag (ID " + canandmag.getAddress().getDeviceId() + "): " + results,
            false
        );
        return false;
    }

    /**
     * Applies settings to a Canandgyro and checks for errors.
     * Will try up to 3 times if settings fail to apply.
     * @param canandgyro The Canandgyro.
     * @param settings The settings to apply.
     * @return {@code true} if success, {@code false} otherwise.
     */
    public static boolean applySettings(Canandgyro canandgyro, CanandgyroSettings settings) {
        return applySettings(canandgyro, settings, 3);
    }

    /**
     * Applies settings to a Canandgyro and checks for errors.
     * @param canandgyro The Canandgyro.
     * @param settings The settings to apply.
     * @param maxTries The number of times to try to apply settings before failing. {@code 1} only tries once.
     * @return {@code true} if success, {@code false} otherwise.
     */
    public static boolean applySettings(Canandgyro canandgyro, CanandgyroSettings settings, int maxTries) {
        if (RobotBase.isSimulation()) return true;

        String results = "";
        for (int i = 0; i < maxTries; i++) {
            if (!canandgyro.setSettings(settings, 0.05)) {
                results += (results.isEmpty() ? "" : ", ") + "setSettings() Failure";
                continue;
            }

            boolean failed = false;
            Map<Integer, Long> device = canandgyro.getSettings(0.5).getFilteredMap();
            for (var entry : settings.getFilteredMap().entrySet()) {
                if (device.get(entry.getKey()) != entry.getValue()) {
                    results += (results.isEmpty() ? "" : ", ") + "Failed check";
                    failed = true;
                    break;
                }
            }

            if (!failed) return true;
        }

        DriverStation.reportError(
            "[ReduxUtil] Canandgyro (ID " + canandgyro.getAddress().getDeviceId() + "): " + results,
            false
        );
        return false;
    }

    /**
     * Performs latency compensation on the X component of the provided 3D vector frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateVec3X(Vec3Frame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getX();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the Y component of the provided 3D vector frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateVec3Y(Vec3Frame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getY();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the Z component of the provided 3D vector frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateVec3Z(Vec3Frame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getZ();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the yaw of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateQuaternionYaw(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getYaw();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the pitch of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateQuaternionPitch(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getPitch();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the roll of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensateQuaternionRoll(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getRoll();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensate(DoubleFrame<Double> frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getData();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensate(Frame<Double> frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getValue();
            timestamp = frame.getTimestamp();
        }

        return latencyCompensate(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param value The value of the frame.
     * @param timestamp The timestamp of the frame.
     * @param slope The rate of change of the frame's data.
     */
    public static double latencyCompensate(double value, double timestamp, double slope) {
        double latency = MathUtil.clamp(Timer.getFPGATimestamp() - timestamp, 0.0, 0.2);
        return value + (slope * latency);
    }
}
