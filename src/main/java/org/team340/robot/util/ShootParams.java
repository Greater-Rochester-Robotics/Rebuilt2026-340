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
            new DataPoint(1.554, 4.0, 31.0),
            new DataPoint(2.158, 7.0, 33.0),
            new DataPoint(2.620, 7.8, 34.0),
            new DataPoint(3.098, 9.9, 36.0),
            new DataPoint(3.602, 11.2, 39.0),
            new DataPoint(4.097, 12.5, 40.0),
            new DataPoint(4.587, 12.5, 44.0)
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
            new DataPoint(1.638, 12.5, 18.0),
            new DataPoint(1.994, 11.5, 20.0),
            new DataPoint(3.23, 10.0, 28.0),
            new DataPoint(4.053, 10.5, 34.0),
            new DataPoint(5.016, 10.75, 44.0),
            new DataPoint(6.001, 11.5, 47.0),
            new DataPoint(6.905, 11.5, 50.0),
            new DataPoint(8.914, 11.5, 60.0),
            new DataPoint(9.995, 12.0, 60.0),
            new DataPoint(11.040, 12.5, 60.0)
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
