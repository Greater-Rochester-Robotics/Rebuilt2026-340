package org.team340.lib.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A simple mutable object that stores a value.
 *
 * <p>Useful for declaring primitives in an enclosing scope to be mutated
 * inside a lambda, as lambdas prohibit capturing non final variables,
 * with the exception of instance variables. This use case however should
 * be executed with caution, as a race condition may occur if the lambda
 * escapes its capturing thread.
 *
 * <p>Fortunately for our purposes, using this class in a command factory
 * to provide stateful behavior does not suffer from aforementioned race
 * conditions as commands are invoked synchronously.
 */
public class Mutable<T> implements Supplier<T>, Consumer<T> {

    public T value;

    /**
     * Create the mutable object.
     * @param value The initial value to store.
     */
    public Mutable(T value) {
        this.value = value;
    }

    /**
     * Gets the current value.
     * @return The current value.
     */
    @Override
    public T get() {
        return value;
    }

    /**
     * Sets a new value.
     * @param value The new value.
     */
    @Override
    public void accept(T value) {
        this.value = value;
    }
}
