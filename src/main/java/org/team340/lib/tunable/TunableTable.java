package org.team340.lib.tunable;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;
import org.team340.lib.tunable.Tunables.Tunable;
import org.team340.lib.tunable.Tunables.TunableBoolean;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.tunable.Tunables.TunableFloat;
import org.team340.lib.tunable.Tunables.TunableHandler;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.lib.tunable.Tunables.TunableString;

/**
 * A TunableTable represents a nested table of tunable values in the NetworkTables tree.
 */
public final class TunableTable {

    private final String name;

    /**
     * Creates a new Tunable table.
     * @param name The name of the table. Expected to be
     *             suffixed with a path separator ("/").
     */
    TunableTable(String name) {
        this.name = name;
    }

    /**
     * Gets a table that can be used to add nested
     * tunable values under a specified path.
     * @param name The name of the table. Must be unique.
     */
    public TunableTable getNested(String name) {
        return Tunables.getNested(this.name + name);
    }

    /**
     * Adds an object to be tuned. The specified object must implement {@link Tunable},
     * or have a registered {@link TunableHandler} in order for it to be added.
     * @param <T> The object's class.
     * @param name The name for the tunable. Must be unique.
     * @param obj The object to be tuned.
     * @return The provided object.
     */
    public <T> T add(String name, T obj) {
        return Tunables.add(this.name + name, obj);
    }

    /**
     * Adds a tunable boolean value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public TunableBoolean value(String name, boolean defaultValue) {
        return Tunables.value(this.name + name, defaultValue);
    }

    /**
     * Adds a tunable boolean value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public TunableBoolean value(String name, boolean defaultValue, BooleanConsumer onChange) {
        return Tunables.value(this.name + name, defaultValue, onChange);
    }

    /**
     * Adds a tunable integer value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public TunableInteger value(String name, int defaultValue) {
        return Tunables.value(this.name + name, defaultValue);
    }

    /**
     * Adds a tunable integer value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public TunableInteger value(String name, int defaultValue, IntConsumer onChange) {
        return Tunables.value(this.name + name, defaultValue, onChange);
    }

    /**
     * Adds a tunable float value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public TunableFloat value(String name, float defaultValue) {
        return Tunables.value(this.name + name, defaultValue);
    }

    /**
     * Adds a tunable float value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public TunableFloat value(String name, float defaultValue, FloatConsumer onChange) {
        return Tunables.value(this.name + name, defaultValue, onChange);
    }

    /**
     * Adds a tunable double value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public TunableDouble value(String name, double defaultValue) {
        return Tunables.value(this.name + name, defaultValue);
    }

    /**
     * Adds a tunable double value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public TunableDouble value(String name, double defaultValue, DoubleConsumer onChange) {
        return Tunables.value(this.name + name, defaultValue, onChange);
    }

    /**
     * Adds a tunable string value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public TunableString value(String name, String defaultValue) {
        return Tunables.value(this.name + name, defaultValue);
    }

    /**
     * Adds a tunable string value to the table.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public TunableString value(String name, String defaultValue, Consumer<String> onChange) {
        return Tunables.value(this.name + name, defaultValue, onChange);
    }
}
