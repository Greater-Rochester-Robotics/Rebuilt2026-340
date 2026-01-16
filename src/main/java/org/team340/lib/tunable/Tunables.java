package org.team340.lib.tunable;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.team340.lib.logging.LoggedRobot;

/**
 * The Tunables class is used to construct tunable properties
 * of the robot to be modified live via NetworkTables.
 */
public final class Tunables {

    /**
     * Base interface for tunable objects.
     */
    @FunctionalInterface
    public static interface Tunable {
        /**
         * Initializes the object to be tuned. This method is
         * called when adding the object to a {@link TunableTable}.
         * @param table The table the object is being added to.
         */
        public void initTunable(TunableTable table);
    }

    /**
     * Interface for wrapping objects to be tunable. Instances of this interface should
     * be passed to {@link Tunables#registerHandler(Class, TunableHandler)} in order for
     * objects of the handler's type to be processed.
     */
    @FunctionalInterface
    public static interface TunableHandler<T> {
        /**
         * This method should implement the setup for an object to be tunable.
         * @param table The table to object is being added to.
         * @param obj The object to make tunable.
         */
        public void init(TunableTable table, T obj);
    }

    /**
     * Represents a tunable value in NetworkTables.
     */
    private static interface TunableValue {
        /**
         * Polls changes from NetworkTables for updates.
         */
        public void poll();
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("/Tunables");

    private static final Map<String, TunableTable> tables = new HashMap<>();
    private static final Map<String, TunableValue> values = new HashMap<>();
    private static final Map<Class<?>, TunableHandler<?>> handlers = new HashMap<>();

    static {
        ThirdParty.registerAll();
    }

    /**
     * Polls changes from NetworkTables. Must be called periodically in order for
     * Tunables to function (this is already done if utilizing {@link LoggedRobot}).
     */
    public static void update() {
        values.values().forEach(TunableValue::poll);
    }

    /**
     * Registers a handler for tuning objects.
     * @param <T> The object's class.
     * @param clazz The object's class.
     * @param handler The handler for the object.
     */
    public static <T> void registerHandler(Class<T> clazz, TunableHandler<T> handler) {
        handlers.put(clazz, handler);
    }

    /**
     * Gets a table that can be used to add nested
     * tunable values under a specified path.
     * @param name The name of the table.
     */
    public static TunableTable getNested(String name) {
        name = normalizeKey(name, true);

        TunableTable existing = tables.get(name);
        if (existing != null) return existing;

        var table = new TunableTable(name);
        tables.put(name, table);
        return table;
    }

    /**
     * Adds an object to be tuned. The specified object must implement {@link Tunable},
     * or have a registered {@link TunableHandler} in order for it to be added.
     * @param <T> The object's class.
     * @param name The name for the tunable. Must be unique.
     * @param obj The object to be tuned.
     * @return The provided object.
     */
    public static <T> T add(String name, T obj) {
        if (obj instanceof Tunable tunable) {
            tunable.initTunable(getNested(name));
        } else {
            boolean initialized = false;
            for (var entry : handlers.entrySet()) {
                if (entry.getKey().isInstance(obj)) {
                    @SuppressWarnings("unchecked")
                    var handler = (TunableHandler<T>) entry.getValue();
                    handler.init(getNested(name), obj);
                    initialized = true;
                    break;
                }
            }

            if (!initialized) {
                DriverStation.reportWarning(
                    "[Tunables] Unable to find TunableHandler for type \"" + obj.getClass().getSimpleName() + "\"",
                    true
                );
            }
        }

        return obj;
    }

    /**
     * Creates a tunable boolean value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableBoolean value(String name, boolean defaultValue) {
        return value(name, defaultValue, null);
    }

    /**
     * Creates a tunable boolean value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableBoolean value(String name, boolean defaultValue, BooleanConsumer onChange) {
        name = normalizeKey(name, false);
        if (values.get(name) instanceof TunableBoolean v) {
            v.addListener(onChange);
            return v;
        }

        var value = new TunableBoolean(name, defaultValue);
        value.addListener(onChange);
        values.put(name, value);

        return value;
    }

    /**
     * A tunable boolean value. Can be modified via NetworkTables.
     */
    public static final class TunableBoolean implements TunableValue, BooleanSupplier, BooleanConsumer {

        private final BooleanEntry entry;
        private final List<BooleanConsumer> listeners = new ArrayList<>();

        private boolean value;

        private TunableBoolean(String name, boolean defaultValue) {
            entry = nt.getBooleanTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;
        }

        /**
         * Returns the value of the tunable.
         */
        public boolean get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(boolean value) {
            this.value = value;
            entry.set(value);
        }

        /**
         * Adds a listener that will be invoked when the tunable's value
         * is modified. Will no-op if the listener is {@code null}.
         * @param onChange A consumer that is invoked when the value of the tunable is modified.
         */
        public void addListener(BooleanConsumer onChange) {
            if (onChange == null) return;
            listeners.add(onChange);
        }

        @Override
        public void poll() {
            if (value != (value = entry.get())) {
                for (var listener : listeners) listener.accept(value);
            }
        }

        @Override
        public boolean getAsBoolean() {
            return get();
        }

        @Override
        public void accept(boolean value) {
            set(value);
        }
    }

    /**
     * Creates a tunable integer value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableInteger value(String name, int defaultValue) {
        return value(name, defaultValue, (IntConsumer) null);
    }

    /**
     * Creates a tunable integer value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableInteger value(String name, int defaultValue, IntConsumer onChange) {
        name = normalizeKey(name, false);
        if (values.get(name) instanceof TunableInteger v) {
            v.addListener(onChange);
            return v;
        }

        var value = new TunableInteger(name, defaultValue);
        value.addListener(onChange);
        values.put(name, value);

        return value;
    }

    /**
     * A tunable integer value. Can be modified via NetworkTables.
     */
    public static final class TunableInteger implements TunableValue, IntSupplier, IntConsumer {

        private final IntegerEntry entry;
        private final List<IntConsumer> listeners = new ArrayList<>();

        private int value;

        private TunableInteger(String name, int defaultValue) {
            entry = nt.getIntegerTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;
        }

        /**
         * Returns the value of the tunable.
         */
        public int get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(int value) {
            this.value = value;
            entry.set(value);
        }

        /**
         * Adds a listener that will be invoked when the tunable's value
         * is modified. Will no-op if the listener is {@code null}.
         * @param onChange A consumer that is invoked when the value of the tunable is modified.
         */
        public void addListener(IntConsumer onChange) {
            if (onChange == null) return;
            listeners.add(onChange);
        }

        @Override
        public void poll() {
            if (value != (value = (int) entry.get())) {
                for (var listener : listeners) listener.accept(value);
            }
        }

        @Override
        public int getAsInt() {
            return get();
        }

        @Override
        public void accept(int value) {
            set(value);
        }
    }

    /**
     * Creates a tunable float value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableFloat value(String name, float defaultValue) {
        return value(name, defaultValue, (FloatConsumer) null);
    }

    /**
     * Creates a tunable float value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableFloat value(String name, float defaultValue, FloatConsumer onChange) {
        name = normalizeKey(name, false);
        if (values.get(name) instanceof TunableFloat v) {
            v.addListener(onChange);
            return v;
        }

        var value = new TunableFloat(name, defaultValue);
        value.addListener(onChange);
        values.put(name, value);

        return value;
    }

    /**
     * A tunable float value. Can be modified via NetworkTables.
     */
    public static final class TunableFloat implements TunableValue, FloatSupplier, FloatConsumer {

        private final FloatEntry entry;
        private final List<FloatConsumer> listeners = new ArrayList<>();

        private float value;

        private TunableFloat(String name, float defaultValue) {
            entry = nt.getFloatTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;
        }

        /**
         * Returns the value of the tunable.
         */
        public float get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(float value) {
            this.value = value;
            entry.set(value);
        }

        /**
         * Adds a listener that will be invoked when the tunable's value
         * is modified. Will no-op if the listener is {@code null}.
         * @param onChange A consumer that is invoked when the value of the tunable is modified.
         */
        public void addListener(FloatConsumer onChange) {
            if (onChange == null) return;
            listeners.add(onChange);
        }

        @Override
        public void poll() {
            if (value != (value = entry.get())) {
                for (var listener : listeners) listener.accept(value);
            }
        }

        @Override
        public float getAsFloat() {
            return get();
        }

        @Override
        public void accept(float value) {
            set(value);
        }
    }

    /**
     * Creates a tunable double value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableDouble value(String name, double defaultValue) {
        return value(name, defaultValue, (DoubleConsumer) null);
    }

    /**
     * Creates a tunable double value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableDouble value(String name, double defaultValue, DoubleConsumer onChange) {
        name = normalizeKey(name, false);
        if (values.get(name) instanceof TunableDouble v) {
            v.addListener(onChange);
            return v;
        }

        var value = new TunableDouble(name, defaultValue);
        value.addListener(onChange);
        values.put(name, value);

        return value;
    }

    /**
     * A tunable double value. Can be modified via NetworkTables.
     */
    public static final class TunableDouble implements TunableValue, DoubleSupplier, DoubleConsumer {

        private final DoubleEntry entry;
        private final List<DoubleConsumer> listeners = new ArrayList<>();

        private double value;

        private TunableDouble(String name, double defaultValue) {
            entry = nt.getDoubleTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;
        }

        /**
         * Returns the value of the tunable.
         */
        public double get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(double value) {
            this.value = value;
            entry.set(value);
        }

        /**
         * Adds a listener that will be invoked when the tunable's value
         * is modified. Will no-op if the listener is {@code null}.
         * @param onChange A consumer that is invoked when the value of the tunable is modified.
         */
        public void addListener(DoubleConsumer onChange) {
            if (onChange == null) return;
            listeners.add(onChange);
        }

        @Override
        public void poll() {
            if (value != (value = entry.get())) {
                for (var listener : listeners) listener.accept(value);
            }
        }

        @Override
        public double getAsDouble() {
            return get();
        }

        @Override
        public void accept(double value) {
            set(value);
        }
    }

    /**
     * Creates a tunable string value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableString value(String name, String defaultValue) {
        return value(name, defaultValue, null);
    }

    /**
     * Creates a tunable string value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableString value(String name, String defaultValue, Consumer<String> onChange) {
        name = normalizeKey(name, false);
        if (values.get(name) instanceof TunableString v) {
            v.addListener(onChange);
            return v;
        }

        var value = new TunableString(name, defaultValue);
        value.addListener(onChange);
        values.put(name, value);

        return value;
    }

    /**
     * A tunable string value. Can be modified via NetworkTables.
     */
    public static final class TunableString implements TunableValue, Supplier<String>, Consumer<String> {

        private final StringEntry entry;
        private final List<Consumer<String>> listeners = new ArrayList<>();

        private String value;

        private TunableString(String name, String defaultValue) {
            entry = nt.getStringTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;
        }

        /**
         * Returns the value of the tunable.
         */
        @Override
        public String get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(String value) {
            this.value = value;
            entry.set(value);
        }

        /**
         * Adds a listener that will be invoked when the tunable's value
         * is modified. Will no-op if the listener is {@code null}.
         * @param onChange A consumer that is invoked when the value of the tunable is modified.
         */
        public void addListener(Consumer<String> onChange) {
            if (onChange == null) return;
            listeners.add(onChange);
        }

        @Override
        public void poll() {
            if (value != (value = entry.get())) {
                for (var listener : listeners) listener.accept(value);
            }
        }

        @Override
        public void accept(String value) {
            set(value);
        }
    }

    /**
     * Normalizes a key for a tunable table or value.
     * @param key The key to normalize.
     * @param suffixSlash If the key should be suffixed with a slash.
     * @return The normalized key.
     */
    private static String normalizeKey(String key, boolean suffixSlash) {
        key = NetworkTable.normalizeKey(key, false);
        if (suffixSlash && !key.endsWith("/")) key += "/";
        return key;
    }
}
