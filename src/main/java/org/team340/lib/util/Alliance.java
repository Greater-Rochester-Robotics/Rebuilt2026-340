package org.team340.lib.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Shorthand for getting the robot's alliance.
 * Defaults to the blue alliance if the current alliance is unknown.
 */
public final class Alliance {

    private Alliance() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Returns {@code true} if the robot is on the blue alliance, and {@code false}
     * if the robot is on the red alliance. If the robot's alliance is unknown,
     * defaults to {@code true} (blue).
     */
    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue);
    }

    /**
     * Returns {@code true} if the robot is on the red alliance, and {@code false}
     * if the robot is on the blue alliance. If the robot's alliance is unknown,
     * defaults to {@code false} (blue).
     */
    public static boolean isRed() {
        return !isBlue();
    }
}
