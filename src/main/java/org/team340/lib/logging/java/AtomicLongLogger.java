package org.team340.lib.logging.java;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.concurrent.atomic.AtomicLong;

@CustomLoggerFor(AtomicLong.class)
public class AtomicLongLogger extends ClassSpecificLogger<AtomicLong> {

    public AtomicLongLogger() {
        super(AtomicLong.class);
    }

    @Override
    public void update(EpilogueBackend backend, AtomicLong atomicLong) {
        backend.log("", atomicLong.get());
    }
}
