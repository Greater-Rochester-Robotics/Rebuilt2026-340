package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@CustomLoggerFor(CommandScheduler.class)
public class CommandSchedulerLogger extends ClassSpecificLogger<CommandScheduler> {

    public CommandSchedulerLogger() {
        super(CommandScheduler.class);
    }

    @Override
    public void update(EpilogueBackend backend, CommandScheduler scheduler) {
        // No-op
    }
}
