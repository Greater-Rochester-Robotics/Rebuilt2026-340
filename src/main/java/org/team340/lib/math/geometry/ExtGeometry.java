package org.team340.lib.math.geometry;

import java.util.function.Supplier;
import org.team340.lib.math.FieldInfo;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables.Tunable;
import org.team340.lib.util.Alliance;

/**
 * Base class for geometry extensions.
 */
abstract class ExtGeometry<T> implements Tunable, Supplier<T> {

    protected T original;
    protected T overWidth;
    protected T overLength;
    protected T overDiagonal;

    /**
     * Creates a geometry extension.
     * @param initialValue The initial blue origin relative value.
     */
    protected ExtGeometry(T initialValue) {
        set(initialValue);
    }

    /**
     * Saves a new value to the container, with all flipped variations. This method is invoked
     * in the constructor, and can also be utilized by {@link #initTunable(TunableTable)}.
     *
     * <p> This method should implement setting the values of {@link #original},
     * {@link #overWidth}, {@link #overLength}, and {@link #overDiagonal}.
     *
     * @param newValue The new blue origin relative value.
     */
    protected abstract void set(T newValue);

    /**
     * Gets the value for the robot's current alliance. Flipping
     * behavior is inherited from {@link FieldInfo#symmetryType()}.
     */
    @Override
    public T get() {
        return get(false);
    }

    /**
     * Gets the value for the robot's current alliance. Flipping
     * behavior is inherited from {@link FieldInfo#symmetryType()}.
     * @param flipWidth If the value should also be flipped over the
     *                  field's width, regardless of the robot's alliance.
     */
    public T get(boolean flipWidth) {
        return get(Alliance.isBlue(), flipWidth);
    }

    /**
     * Gets the value for the specified alliance. Flipping
     * behavior is inherited from {@link FieldInfo#symmetryType()}.
     * @param blue If the blue alliance value should be returned.
     * @param flipWidth If the value should also be flipped over the
     *                  field's width, regardless of the alliance.
     */
    public T get(boolean blue, boolean flipWidth) {
        if (blue) {
            return !flipWidth ? original : overWidth;
        }

        return switch (FieldInfo.symmetryType()) {
            case MIRROR -> !flipWidth ? overLength : overDiagonal;
            case ROTATE -> !flipWidth ? overDiagonal : overLength;
        };
    }

    /**
     * Gets the value for the blue alliance.
     */
    public T getBlue() {
        return getBlue(false);
    }

    /**
     * Gets the value for the blue alliance.
     * @param flipWidth If the value should also be flipped over the field's width.
     */
    public T getBlue(boolean flipWidth) {
        return get(true, flipWidth);
    }

    /**
     * Gets the value for the red alliance.
     */
    public T getRed() {
        return getRed(false);
    }

    /**
     * Gets the value for the red alliance.
     * @param flipWidth If the value should also be flipped over the field's width.
     */
    public T getRed(boolean flipWidth) {
        return get(false, flipWidth);
    }
}
