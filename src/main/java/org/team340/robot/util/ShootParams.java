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
    public static final double TOF = 1.2; // wow 1.2 seconds thats really cool

    /**
     * Our hood position lookup table.
     * Maps hub distance (meters) to hood position (motor rotations).
     */
    public static final InterpolatingDoubleTreeMap hoodPositionMap;

    /**
     * Our shooter velocity lookup table.
     * Maps hub distance (meters) to shooter velocity (rotations/second).
     */
    public static final InterpolatingDoubleTreeMap shooterVelocityMap;

    static {
        // Data obtained from empirical testing.
        final DataPoint[] dataPoints = {
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
        hoodPositionMap = new InterpolatingDoubleTreeMap();
        shooterVelocityMap = new InterpolatingDoubleTreeMap();

        // Populate the tables with our configured data points.
        for (final DataPoint dataPoint : dataPoints) {
            hoodPositionMap.put(dataPoint.distance, dataPoint.hoodPosition);
            shooterVelocityMap.put(dataPoint.distance, dataPoint.shooterVelocity);
        }
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
