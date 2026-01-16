package org.team340.lib.util.vendors;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utilities for the CTRE Phoenix 6 API.
 */
public final class PhoenixUtil {

    private PhoenixUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Disables background processes managed by Phoenix, intended for
     * competition use. Note that Phoenix Tuner X will not be able to
     * connect to the robot when this method is in use.
     */
    public static void disableDaemons() {
        Unmanaged.setPhoenixDiagnosticsStartTime(-1.0);
        SignalLogger.enableAutoLogging(false);
    }

    /**
     * Runs a Phoenix API call and checks for errors. Will
     * try up to 3 times if the target API call fails.
     * @param target The target call to run.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(Supplier<StatusCode> target) {
        return run(target, 3);
    }

    /**
     * Runs a Phoenix API call and checks for errors.
     * @param target The target call to run.
     * @param maxTries The number of times to try the call before failing. {@code 1} only runs the call once.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(Supplier<StatusCode> target, int maxTries) {
        String results = "";
        for (int i = 0; i < maxTries; i++) {
            StatusCode result = target.get();
            if (result.isOK()) return true;
            results += (results.isEmpty() ? "" : ", ") + result.name();
        }

        DriverStation.reportError("[PhoenixUtil] Error running API call: " + results, true);
        return false;
    }
}
