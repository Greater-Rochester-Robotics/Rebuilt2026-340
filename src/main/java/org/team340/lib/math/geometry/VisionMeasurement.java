package org.team340.lib.math.geometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents a measurement from vision to apply to a pose estimator.
 * @see {@link PoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
 */
public final record VisionMeasurement(
    Pose2d pose,
    double timestamp,
    Matrix<N3, N1> stdDevs
) implements Comparable<VisionMeasurement> {
    /**
     * Represents a measurement from vision to apply to a pose estimator.
     * @see {@link PoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public VisionMeasurement(Pose2d pose, double timestamp) {
        this(pose, timestamp, null);
    }

    @Override
    public int compareTo(VisionMeasurement o) {
        return Double.compare(timestamp, o.timestamp());
    }

    @Override
    public String toString() {
        return String.format("VisionMeasurement(%s, timestamp: %.3f)", pose, timestamp);
    }
}
