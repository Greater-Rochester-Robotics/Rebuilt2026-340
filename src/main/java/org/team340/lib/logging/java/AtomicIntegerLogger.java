package org.team340.lib.logging.java;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.concurrent.atomic.AtomicInteger;

@CustomLoggerFor(AtomicInteger.class)
public class AtomicIntegerLogger extends ClassSpecificLogger<AtomicInteger> {

    public AtomicIntegerLogger() {
        super(AtomicInteger.class);
    }

    @Override
    public void update(EpilogueBackend backend, AtomicInteger atomicInteger) {
        backend.log("", atomicInteger.get());
    }
}
