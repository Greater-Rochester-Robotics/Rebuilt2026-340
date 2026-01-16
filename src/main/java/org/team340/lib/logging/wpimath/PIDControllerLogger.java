package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.controller.PIDController;

@CustomLoggerFor(PIDController.class)
public class PIDControllerLogger extends ClassSpecificLogger<PIDController> {

    public PIDControllerLogger() {
        super(PIDController.class);
    }

    @Override
    public void update(EpilogueBackend backend, PIDController pidController) {
        // No-op
    }
}
