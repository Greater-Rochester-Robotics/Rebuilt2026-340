package org.team340.lib.math;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

/**
 * Retrieves information about the playing field from an AprilTag field
 * layout. By default, {@link AprilTagFields#kDefaultField} is used.
 */
public final class FieldInfo {

    private FieldInfo() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Specifies the direction in which the field is symmetric.
     */
    public static enum SymmetryType {
        /** The field is symmetric over its length. */
        MIRROR,
        /** The field is symmetric over its diagonal. */
        ROTATE
    }

    private static AprilTagFieldLayout layout;
    private static SymmetryType symmetryType;

    static {
        setLayout(AprilTagFields.kDefaultField);
    }

    /**
     * Loads a new AprilTag layout to retrieve field information from.
     * This method should be called before all other user code.
     * @param field The loadable AprilTag field layout.
     */
    public static void setLayout(AprilTagFields field) {
        try {
            layout = AprilTagFieldLayout.loadField(field);
            symmetryType = switch (field) {
                case k2026RebuiltWelded -> SymmetryType.ROTATE;
                case k2026RebuiltAndymark -> SymmetryType.ROTATE;
                case k2025ReefscapeWelded -> SymmetryType.ROTATE;
                case k2025ReefscapeAndyMark -> SymmetryType.ROTATE;
                case k2024Crescendo -> SymmetryType.MIRROR;
                case k2023ChargedUp -> SymmetryType.MIRROR;
                case k2022RapidReact -> SymmetryType.ROTATE;
            };
        } catch (Exception e) {
            layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
            symmetryType = SymmetryType.MIRROR;

            DriverStation.reportError(
                "[AprilTags] Unable to load layout from field \"" + field.name() + "\": " + e.getMessage(),
                true
            );
        }
    }

    /**
     * Loads a new AprilTag layout to retrieve field information from.
     * This method should be called before all other user code.
     * @param resourcePath The absolute path of the resource.
     * @param symmetryType The field's symmetry type.
     */
    public static void setLayout(String resourcePath, SymmetryType symmetryType) {
        try {
            layout = AprilTagFieldLayout.loadFromResource(resourcePath);
            FieldInfo.symmetryType = symmetryType;
        } catch (Exception e) {
            layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
            symmetryType = SymmetryType.MIRROR;

            DriverStation.reportError(
                "[AprilTags] Unable to load layout from resource \"" + resourcePath + "\": " + e.getMessage(),
                true
            );
        }
    }

    /**
     * Sets a new AprilTag layout to retrieve field information from.
     * This method should be called before all other user code.
     * @param layout The AprilTag field layout.
     * @param symmetryType The field's symmetry type.
     */
    public static void setLayout(AprilTagFieldLayout layout, SymmetryType symmetryType) {
        FieldInfo.layout = layout;
        FieldInfo.symmetryType = symmetryType;
    }

    /**
     * Returns the length (X-axis) of the field in meters.
     */
    public static double length() {
        return layout.getFieldLength();
    }

    /**
     * Returns the width (Y-axis) of the field in meters.
     */
    public static double width() {
        return layout.getFieldWidth();
    }

    /**
     * Returns the direction in which the field is symmetric.
     */
    public static SymmetryType symmetryType() {
        return symmetryType;
    }

    /**
     * Returns the current field's AprilTag layout.
     */
    public static AprilTagFieldLayout aprilTags() {
        return layout;
    }
}
