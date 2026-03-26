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
    public static final double TOF = 1.0; // wow 1 second thats really cool

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
            new DataPoint(1.51, 0.0, 34.0),
            new DataPoint(2.00, 0.0, 39.0),
            new DataPoint(2.50, 4.0, 41.0),
            new DataPoint(2.99, 7.5, 39.0),
            new DataPoint(3.51, 10.0, 44.0),
            new DataPoint(3.96, 11.95, 46.0),
            new DataPoint(4.59, 12.4, 48.5),
            new DataPoint(5.15, 14.9, 53.0),
            // Outside of alliance zone
            new DataPoint(6.15, 15.0, 58.0),
            new DataPoint(8.39, 12.0, 68.0),
            new DataPoint(9.71, 12.0, 75.0),
            new DataPoint(10.12, 12.0, 87.0)
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
