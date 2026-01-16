package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@CustomLoggerFor(CommandXboxController.class)
public class CommandXboxControllerLogger extends ClassSpecificLogger<CommandXboxController> {

    public CommandXboxControllerLogger() {
        super(CommandXboxController.class);
    }

    @Override
    public void update(EpilogueBackend backend, CommandXboxController profile) {
        // No-op
    }
}
