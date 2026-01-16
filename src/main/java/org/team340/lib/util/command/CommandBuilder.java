package org.team340.lib.util.command;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;

/**
 * A command builder. Very similar to {@link FunctionalCommand}.
 * Stylistic alternative to using decorators.
 */
public class CommandBuilder extends Command {

    private Runnable onInitialize = () -> {};
    private Runnable onExecute = () -> {};
    private BooleanConsumer onEnd = interrupted -> {};
    private BooleanSupplier isFinished = () -> false;

    /**
     * Create the command builder.
     * @param requirements The subsystems required by the command.
     * @param name The command's name.
     */
    public CommandBuilder(String name, Subsystem... requirements) {
        this(requirements);
        setName(name);
    }

    /**
     * Create the command builder.
     * @param requirements The subsystems required by the command.
     */
    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    public CommandBuilder onInitialize(Runnable onInitialize) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onInitialize = onInitialize;
        return this;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    public CommandBuilder onExecute(Runnable onExecute) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onExecute = onExecute;
        return this;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally,
     * or when it interrupted/canceled.
     */
    public CommandBuilder onEnd(Runnable onEnd) {
        return onEnd(interrupted -> onEnd.run());
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally,
     * or when it interrupted/canceled. Supplied boolean is if the command was interrupted.
     */
    public CommandBuilder onEnd(BooleanConsumer onEnd) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onEnd = onEnd;
        return this;
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it. By default, this returns {@code false}. If {@code true}, the command
     * is effectively an {@link InstantCommand}. If {@code false}, the command will run continuously until
     * it is canceled.
     */
    public CommandBuilder isFinished(boolean isFinished) {
        return isFinished(() -> isFinished);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it. By default, this returns {@code false}.
     */
    public CommandBuilder isFinished(BooleanSupplier isFinished) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.isFinished = isFinished;
        return this;
    }

    @Override
    public void initialize() {
        onInitialize.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
