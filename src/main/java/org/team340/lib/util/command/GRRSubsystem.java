package org.team340.lib.util.command;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} implementation.
 */
public abstract class GRRSubsystem implements Subsystem {

    public GRRSubsystem() {
        register();
    }

    /**
     * Creates a command builder that requires this subsystem.
     */
    public CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     * @param name The name of the command.
     */
    public CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }
}
