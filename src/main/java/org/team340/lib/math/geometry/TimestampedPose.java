package org.team340.lib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Contains a {@link Pose2d} alongside a timestamp in seconds.
 */
public final record TimestampedPose(Pose2d pose, double timestamp) implements Comparable<TimestampedPose> {
    @Override
    public int compareTo(TimestampedPose o) {
        return Double.compare(timestamp, o.timestamp());
    }

    @Override
    public String toString() {
        return String.format("TimestampedPose(%s, timestamp: %.3f)", pose, timestamp);
    }
}
