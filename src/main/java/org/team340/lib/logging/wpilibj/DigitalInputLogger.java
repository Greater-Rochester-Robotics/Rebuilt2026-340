package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DigitalInput;

@CustomLoggerFor(DigitalInput.class)
public class DigitalInputLogger extends ClassSpecificLogger<DigitalInput> {

    public DigitalInputLogger() {
        super(DigitalInput.class);
    }

    @Override
    public void update(EpilogueBackend backend, DigitalInput digitalInput) {
        backend.log("", digitalInput.get());
    }
}
