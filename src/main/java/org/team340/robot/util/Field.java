package org.team340.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team340.lib.math.FieldInfo;
import org.team340.lib.math.PAPFController.LateralObstacle;
import org.team340.lib.math.PAPFController.LineObstacle;
import org.team340.lib.math.PAPFController.LongitudinalObstacle;
import org.team340.lib.math.PAPFController.Obstacle;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.math.geometry.ExtTranslation;

public final class Field {

    /** The center of the field over its length (X direction). Also the CENTER LINE. */
    public static final double X_CENTER = FieldInfo.length() / 2.0;
    /** The center of the field over its width (Y direction). */
    public static final double Y_CENTER = FieldInfo.width() / 2.0;

    /** The X coordinate of the edge of the blue ALLIANCE ZONE. */
    public static final double BLUE_ZONE = getTag(26).getX();
    /** The X coordinate of the edge of the red ALLIANCE ZONE. */
    public static final double RED_ZONE = getTag(10).getX();

    public static final double BLUE_TOWER_Y = getTag(31).getY();
    public static final double RED_TOWER_Y = getTag(15).getY();

    // HUB location helpers
    private static final double HUB_WIDTH = Units.inchesToMeters(47.0);
    private static final double HUB_HALF_WIDTH = HUB_WIDTH / 2.0;
    private static final double HUB_NEAR = BLUE_ZONE;
    private static final double HUB_FAR = HUB_NEAR + HUB_WIDTH;

    /** The center of the HUB. */
    public static final ExtTranslation HUB = new ExtTranslation(HUB_NEAR + HUB_HALF_WIDTH, Y_CENTER);

    /** The near left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_NEAR_LEFT_CORNER = new ExtTranslation(HUB_NEAR, Y_CENTER + HUB_HALF_WIDTH);
    /** The near right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_NEAR_RIGHT_CORNER = new ExtTranslation(HUB_NEAR, Y_CENTER - HUB_HALF_WIDTH);
    /** The far left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_FAR_LEFT_CORNER = new ExtTranslation(HUB_FAR, Y_CENTER + HUB_HALF_WIDTH);
    /** The far right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_FAR_RIGHT_CORNER = new ExtTranslation(HUB_FAR, Y_CENTER - HUB_HALF_WIDTH);

    // TOWER location helpers
    private static final double TOWER_LEFT_Y = 4.2;
    private static final double TOWER_RIGHT_Y = 3.33;
    private static final double CLIMB_APPROACH_X = 1.8;
    private static final double CLIMBING_X = 1.485;

    /** The approach location for the left TOWER UPRIGHT. */
    public static final ExtPose TOWER_LEFT_APPROACH = new ExtPose(CLIMB_APPROACH_X, TOWER_LEFT_Y, Rotation2d.kZero);
    /** The climbing location for the left TOWER UPRIGHT. */
    public static final ExtPose TOWER_LEFT_CLIMB = new ExtPose(CLIMBING_X, TOWER_LEFT_Y, Rotation2d.kZero);
    /** The approach location for the right TOWER UPRIGHT. */
    public static final ExtPose TOWER_RIGHT_APPROACH = new ExtPose(CLIMB_APPROACH_X, TOWER_RIGHT_Y, Rotation2d.kZero);
    /** The climbing location for the right TOWER UPRIGHT. */
    public static final ExtPose TOWER_RIGHT_CLIMB = new ExtPose(CLIMBING_X, TOWER_RIGHT_Y, Rotation2d.kZero);

    // TRENCH location helpers
    private static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);
    private static final double TRENCH_BASE_WIDTH = Units.inchesToMeters(12.0);
    private static final double TRENCH_OFFSET = Units.inchesToMeters(96.5);
    private static final double TRENCH_NEAR = BLUE_ZONE;
    private static final double TRENCH_FAR = TRENCH_NEAR + TRENCH_DEPTH;

    // spotless:off

    /** The near BUMP side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_NEAR_BUMP_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER + TRENCH_OFFSET);
    /** The far BUMP side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_FAR_BUMP_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER + TRENCH_OFFSET);
    /** The near opening side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_NEAR_OPENING_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER + TRENCH_OFFSET + TRENCH_BASE_WIDTH);
    /** The far opening side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_FAR_OPENING_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER + TRENCH_OFFSET + TRENCH_BASE_WIDTH);
    /** The near field boundary corner of the left TRENCH, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_NEAR_FIELD_CORNER = new ExtTranslation(TRENCH_NEAR, FieldInfo.width());
    /** The far field boundary corner of the left TRENCH, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_FAR_FIELD_CORNER = new ExtTranslation(TRENCH_FAR, FieldInfo.width());

    /** The near BUMP side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_NEAR_BUMP_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER - TRENCH_OFFSET);
    /** The far BUMP side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_FAR_BUMP_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER - TRENCH_OFFSET);
    /** The near opening side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER - TRENCH_OFFSET - TRENCH_BASE_WIDTH);
    /** The far opening side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_FAR_OPENING_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER - TRENCH_OFFSET - TRENCH_BASE_WIDTH);
    /** The near field boundary corner of the right TRENCH, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_NEAR_FIELD_CORNER = new ExtTranslation(TRENCH_NEAR, 0.0);
    /** The far field boundary corner of the right TRENCH, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_FAR_FIELD_CORNER = new ExtTranslation(TRENCH_FAR, 0.0);

    /** Obstacles for the P-APF. */
    public static final Obstacle[] OBSTACLES = {
        // Field boundary
        new LongitudinalObstacle(0.0, 2.0, 2.0),
        new LongitudinalObstacle(FieldInfo.length(), 2.0, 2.0),
        new LateralObstacle(0.0, 2.0, 2.0),
        new LateralObstacle(FieldInfo.width(), 2.0, 2.0),

        // Blue HUB
        new LineObstacle(HUB_NEAR_LEFT_CORNER.getBlue(), HUB_NEAR_RIGHT_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(HUB_FAR_LEFT_CORNER.getBlue(), HUB_FAR_RIGHT_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(HUB_NEAR_LEFT_CORNER.getBlue(), HUB_FAR_LEFT_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(HUB_NEAR_RIGHT_CORNER.getBlue(), HUB_FAR_RIGHT_CORNER.getBlue(), 2.0, 2.0),

        // Red HUB
        new LineObstacle(HUB_NEAR_LEFT_CORNER.getRed(), HUB_NEAR_RIGHT_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(HUB_FAR_LEFT_CORNER.getRed(), HUB_FAR_RIGHT_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(HUB_NEAR_LEFT_CORNER.getRed(), HUB_FAR_LEFT_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(HUB_NEAR_RIGHT_CORNER.getRed(), HUB_FAR_RIGHT_CORNER.getRed(), 2.0, 2.0),

        // Blue left TRENCH base
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_BUMP_CORNER.getBlue(), LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_FAR_BUMP_CORNER.getBlue(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_BUMP_CORNER.getBlue(), LEFT_TRENCH_BASE_FAR_BUMP_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),

        // Blue right TRENCH base
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_BUMP_CORNER.getBlue(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_FAR_BUMP_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_BUMP_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_BUMP_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),

        // Red left TRENCH base
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_BUMP_CORNER.getRed(), LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_FAR_BUMP_CORNER.getRed(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_BUMP_CORNER.getRed(), LEFT_TRENCH_BASE_FAR_BUMP_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),

        // Red right TRENCH base
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_BUMP_CORNER.getRed(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_FAR_BUMP_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_BUMP_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_BUMP_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),

        // Our robot cannot fit under the TRENCH, so block off the TRENCH openings as well.

        // Blue TRENCH openings
        new LineObstacle(LEFT_TRENCH_NEAR_FIELD_CORNER.getBlue(), LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_FAR_FIELD_CORNER.getBlue(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_NEAR_FIELD_CORNER.getBlue(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_FAR_FIELD_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 2.0, 2.0),

        // Red TRENCH openings
        new LineObstacle(LEFT_TRENCH_NEAR_FIELD_CORNER.getRed(), LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(LEFT_TRENCH_FAR_FIELD_CORNER.getRed(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_NEAR_FIELD_CORNER.getRed(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), 2.0, 2.0),
        new LineObstacle(RIGHT_TRENCH_FAR_FIELD_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 2.0, 2.0),
    };

    // spotless:on

    /**
     * Returns the pose of the specified AprilTag on the field.
     * @param id The ID of the tag.
     * @return A {@link Pose3d} representing the tag's blue origin relative field position.
     */
    private static Pose3d getTag(int id) {
        return FieldInfo.aprilTags().getTagPose(id).get();
    }

    private Field() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
