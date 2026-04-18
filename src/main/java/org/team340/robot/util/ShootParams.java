package org.team340.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Shooter;

/**
 * This class stores our lookup tables for the {@link Hood} and {@link Shooter}
 * subsystems to utilize when converting hub distance to motor setpoints.
 */
public final class ShootParams {

    /**
     * The ball's time of flight, in seconds. All entries in
     * our lookup tables below are tuned for this constant.
     */
    public static final double HUB_TOF = 1.2; // wow 1.2 seconds thats really cool

    /**
     * Our hood position lookup table for shooting.
     * Maps hub distance (meters) to hood position (motor rotations).
     */
    private static final InterpolatingDoubleTreeMap hubHoodPositionMap;

    /**
     * Our shooter velocity lookup table for shooting.
     * Maps hub distance (meters) to shooter velocity (rotations/second).
     */
    private static final InterpolatingDoubleTreeMap hubShooterVelocityMap;

    /**
     * Our hood position lookup table for ferrying.
     * Maps ferry distance (meters) to hood position (motor rotations).
     */
    private static final InterpolatingDoubleTreeMap ferryingHoodPositionMap;

    /**
     * Our shooter velocity lookup table for ferrying.
     * Maps ferry distance (meters) to shooter velocity (rotations/second).
     */
    private static final InterpolatingDoubleTreeMap ferryingShooterVelocityMap;

    static {
        // Data obtained from empirical testing.
        final DataPoint[] hubDataPoints = {
            new DataPoint(4.95, 8.0, 44.5), // Bad TOF
            new DataPoint(4.45, 7.75, 41.0),
            new DataPoint(3.91, 7.5, 39.5),
            new DataPoint(3.41, 7.6, 38.0), // Bad TOF
            new DataPoint(2.90, 7.5, 36.25),
            new DataPoint(2.50, 7.3, 34.5),
            new DataPoint(2.08, 6.9, 34.0),
            new DataPoint(1.52, 3.8, 34.0)
        };

        // Create our lookup tables.
        hubHoodPositionMap = new InterpolatingDoubleTreeMap();
        hubShooterVelocityMap = new InterpolatingDoubleTreeMap();

        // Populate the tables with our configured data points.
        for (final DataPoint dataPoint : hubDataPoints) {
            hubHoodPositionMap.put(dataPoint.distance, dataPoint.hoodPosition);
            hubShooterVelocityMap.put(dataPoint.distance, dataPoint.shooterVelocity);
        }
    }

    static {
        // Data obtained from empirical testing.
        final DataPoint[] ferryingDataPoints = {
            new DataPoint(4.95, 8.0, 44.5), // Bad TOF
            new DataPoint(4.45, 7.75, 41.0),
            new DataPoint(3.91, 7.5, 39.5),
            new DataPoint(3.41, 7.6, 38.0), // Bad TOF
            new DataPoint(2.90, 7.5, 36.25),
            new DataPoint(2.50, 7.3, 34.5),
            new DataPoint(2.08, 6.9, 34.0),
            new DataPoint(1.52, 3.8, 34.0)
        };

        // Create our lookup tables.
        ferryingHoodPositionMap = new InterpolatingDoubleTreeMap();
        ferryingShooterVelocityMap = new InterpolatingDoubleTreeMap();

        // Populate the tables with our configured data points.
        for (final DataPoint dataPoint : ferryingDataPoints) {
            ferryingHoodPositionMap.put(dataPoint.distance, dataPoint.hoodPosition);
            ferryingShooterVelocityMap.put(dataPoint.distance, dataPoint.shooterVelocity);
        }
    }

    /**
     * Use hood position maps to get the hood position for the supplied distance and zone.
     * @param distance The robot's distance to the hub, in meters.
     * @param inOurZone If we are in our zone.
     * @return the hood position in radians.
     */
    public static double getHoodPosition(double distance, boolean inOurZone) {
        return (inOurZone ? hubHoodPositionMap : ferryingHoodPositionMap).get(distance);
    }

    /**
     * Use hood position maps to get the shooter velocity for the supplied distance and zone.
     * @param distance The robot's distance to the hub, in meters.
     * @param inOurZone If we are in our zone.
     * @return the shooter velocity in radians.
     */
    public static double getShooterVelocity(double distance, boolean inOurZone) {
        return (inOurZone ? hubShooterVelocityMap : ferryingShooterVelocityMap).get(distance);
    }

    /**
     * A data point for our lookup tables. Contains the hood position
     * and shooter velocity needed to make a shot from a given distance.
     * @param distance The robot's distance to the hub, in meters.
     * @param hoodPosition The position of the hood, in motor rotations.
     * @param shooterVelocity The velocity of the shooter wheels, in rotations/second.
     */
    private record DataPoint(double distance, double hoodPosition, double shooterVelocity) {}

    private ShootParams() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
