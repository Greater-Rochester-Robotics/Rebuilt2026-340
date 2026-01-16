package org.team340.lib.math.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team340.lib.math.FieldFlip;
import org.team340.lib.tunable.TunableTable;

/**
 * A container for {@link Rotation2d} objects that enables live tuning via
 * NetworkTables, and dynamically returns flipped variants of the original
 * rotation based on the robot's current alliance via {@link ExtRotation#get()}.
 */
public final class ExtRotation extends ExtGeometry<Rotation2d> {

    /**
     * Constructs and returns an extended {@link Rotation2d} with the given radian value.
     * @param radians The blue origin relative value of the angle in radians.
     */
    public static ExtRotation fromRadians(double radians) {
        return new ExtRotation(Rotation2d.fromRadians(radians));
    }

    /**
     * Constructs and returns an extended {@link Rotation2d} with the given degree value.
     * @param degrees The blue origin relative value of the angle in degrees.
     */
    public static ExtRotation fromDegrees(double degrees) {
        return new ExtRotation(Rotation2d.fromDegrees(degrees));
    }

    /**
     * Constructs and returns an extended {@link Rotation2d} with the given rotations value.
     * @param rotations The blue origin relative value of the angle in rotations.
     */
    public static ExtRotation fromRotations(double rotations) {
        return new ExtRotation(Rotation2d.fromRotations(rotations));
    }

    /**
     * Creates an extended {@link Rotation2d}.
     * @param value The blue origin relative value of the angle in radians.
     */
    public ExtRotation(double value) {
        this(new Rotation2d(value));
    }

    /**
     * Creates an extended {@link Rotation2d}.
     * @param x The blue origin relative x component or cosine of the rotation.
     * @param y The blue origin relative y component or sine of the rotation.
     */
    public ExtRotation(double x, double y) {
        this(new Rotation2d(x, y));
    }

    /**
     * Creates an extended {@link Rotation2d}.
     * @param rotation The blue origin relative rotation.
     */
    public ExtRotation(Rotation2d rotation) {
        super(rotation);
    }

    @Override
    protected void set(Rotation2d newValue) {
        original = newValue;
        overWidth = FieldFlip.overWidth(newValue);
        overLength = FieldFlip.overLength(newValue);
        overDiagonal = FieldFlip.overDiagonal(newValue);
    }

    @Override
    public void initTunable(TunableTable table) {
        table.value("degrees", original.getDegrees(), v -> set(Rotation2d.fromDegrees(v)));
    }
}
