package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.controller.ProfiledPIDController;

@CustomLoggerFor(ProfiledPIDController.class)
public class ProfiledPIDControllerLogger extends ClassSpecificLogger<ProfiledPIDController> {

    public ProfiledPIDControllerLogger() {
        super(ProfiledPIDController.class);
    }

    @Override
    public void update(EpilogueBackend backend, ProfiledPIDController profiledPIDController) {
        // No-op
    }
}
