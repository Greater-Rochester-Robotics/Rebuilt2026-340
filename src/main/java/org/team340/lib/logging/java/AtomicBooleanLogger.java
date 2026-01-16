package org.team340.lib.logging.java;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.concurrent.atomic.AtomicBoolean;

@CustomLoggerFor(AtomicBoolean.class)
public class AtomicBooleanLogger extends ClassSpecificLogger<AtomicBoolean> {

    public AtomicBooleanLogger() {
        super(AtomicBoolean.class);
    }

    @Override
    public void update(EpilogueBackend backend, AtomicBoolean atomicBoolean) {
        backend.log("", atomicBoolean.get());
    }
}
