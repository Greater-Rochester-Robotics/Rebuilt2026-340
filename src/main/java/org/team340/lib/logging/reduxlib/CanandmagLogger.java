package org.team340.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagStatus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Canandmag.class)
public class CanandmagLogger extends ClassSpecificLogger<Canandmag> {

    public CanandmagLogger() {
        super(Canandmag.class);
    }

    @Override
    public void update(EpilogueBackend backend, Canandmag canandmag) {
        backend.log("absolutePosition", canandmag.getAbsPosition());
        backend.log("magnetInRange", canandmag.magnetInRange());
        backend.log("position", canandmag.getPosition());
        backend.log("status", canandmag.getStatus(), CanandmagStatus.struct);
    }
}
