package org.team340.lib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team340.lib.math.FieldFlip;
import org.team340.lib.tunable.TunableTable;

/**
 * A container for {@link Pose2d} objects that enables live tuning via
 * NetworkTables, and dynamically returns flipped variants of the original
 * pose based on the robot's current alliance via {@link ExtPose#get()}.
 */
public final class ExtPose extends ExtGeometry<Pose2d> {

    /**
     * Creates an extended {@link Pose2d}. All parameters are expected to be blue origin relative.
     * @param x The x component of the translational component of the pose.
     * @param y The y component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public ExtPose(double x, double y, Rotation2d rotation) {
        this(new Pose2d(x, y, rotation));
    }

    /**
     * Creates an extended {@link Pose2d}. All parameters are expected to be blue origin relative.
     * @param translation The translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public ExtPose(Translation2d translation, Rotation2d rotation) {
        this(new Pose2d(translation, rotation));
    }

    /**
     * Creates an extended {@link Pose2d}.
     * @param pose The blue origin relative pose.
     */
    public ExtPose(Pose2d pose) {
        super(pose);
    }

    @Override
    protected void set(Pose2d newValue) {
        original = newValue;
        overWidth = FieldFlip.overWidth(newValue);
        overLength = FieldFlip.overLength(newValue);
        overDiagonal = FieldFlip.overDiagonal(newValue);
    }

    @Override
    public void initTunable(TunableTable table) {
        table.value("x", original.getX(), v -> set(new Pose2d(v, original.getY(), original.getRotation())));
        table.value("y", original.getY(), v -> set(new Pose2d(original.getX(), v, original.getRotation())));
        table.value("degrees", original.getRotation().getDegrees(), v ->
            set(new Pose2d(original.getX(), original.getY(), Rotation2d.fromDegrees(v)))
        );
    }
}
