package org.team340.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShootParams {

    public static final double TOF = 1.0; // Time of Flight

    public static final InterpolatingDoubleTreeMap hoodPositionMap;
    public static final InterpolatingDoubleTreeMap shootersVelocityMap;

    static {
        // spotless:off
        final DataPoint[] dataPoints = {
            new DataPoint(1.51, 0.0, 34.0),
            new DataPoint(2.00, 0.0, 39.0),
            new DataPoint(2.50, 4.0, 41.0),
            new DataPoint(2.99, 7.5, 39.0),
            new DataPoint(3.51, 10.0, 44.0),
            new DataPoint(3.96, 12.0, 46.0),
            new DataPoint(4.59, 12.5, 48.0),
            new DataPoint(5.15, 15.0, 52.0),
        };
        // spotless:on

        hoodPositionMap = new InterpolatingDoubleTreeMap();
        shootersVelocityMap = new InterpolatingDoubleTreeMap();

        for (final DataPoint dataPoint : dataPoints) {
            hoodPositionMap.put(dataPoint.distance, dataPoint.hoodPosition);
            shootersVelocityMap.put(dataPoint.distance, dataPoint.shootersVelocity);
        }
    }

    private record DataPoint(double distance, double hoodPosition, double shootersVelocity) {}

    private ShootParams() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
