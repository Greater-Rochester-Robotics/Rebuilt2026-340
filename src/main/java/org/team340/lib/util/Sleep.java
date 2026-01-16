package org.team340.lib.util;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility class for sleeping the current thread,
 * with the option to skip if in simulation.
 */
public final class Sleep {

    private Sleep() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Sleeps the thread for the specified duration.
     * Will always run, even if in simulation.
     * @param time The time to sleep for.
     */
    public static void of(Time time) {
        of(time, false);
    }

    /**
     * Sleeps the thread for the specified duration.
     * @param time The time to sleep for.
     * @param skipSim If the thread should skip sleeping if in simulation.
     */
    public static void of(Time time, boolean skipSim) {
        ms(time.in(Milliseconds), skipSim);
    }

    /**
     * Sleeps the thread for the specified duration in seconds.
     * Will always run, even if in simulation.
     * @param seconds The time to sleep for in seconds.
     */
    public static void seconds(double seconds) {
        seconds(seconds, false);
    }

    /**
     * Sleeps the thread for the specified duration in seconds.
     * @param seconds The time to sleep for in seconds.
     * @param skipSim If the thread should skip sleeping if in simulation.
     */
    public static void seconds(double seconds, boolean skipSim) {
        ms(seconds * 1000.0, skipSim);
    }

    /**
     * Sleeps the thread for the specified duration in milliseconds.
     * Will always run, even if in simulation.
     * @param ms The time to sleep for in milliseconds.
     */
    public static void ms(double ms) {
        ms(ms, false);
    }

    /**
     * Sleeps the thread for the specified duration in milliseconds.
     * @param ms The time to sleep for in milliseconds.
     * @param skipSim If the thread should skip sleeping if in simulation.
     */
    public static void ms(double ms, boolean skipSim) {
        if (!skipSim || !RobotBase.isSimulation()) {
            try {
                Thread.sleep((long) ms, (int) Math.min(Math.round((ms % 1.0) * 1e6), 1e6 - 1.0));
            } catch (Exception e) {}
        }
    }
}
