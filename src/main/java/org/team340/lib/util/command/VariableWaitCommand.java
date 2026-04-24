package org.team340.lib.util.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * A command that does nothing but takes a specified and tunable amount of time to finish.
 *
 */
public class VariableWaitCommand extends Command {

    /** The timer used for waiting. */
    protected Timer timer = new Timer();

    private DoubleSupplier duration;

    /**
     * Creates a new TunableWaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param duration default time to wait, in seconds.
     */
    public VariableWaitCommand(DoubleSupplier duration) {
        this.duration = duration;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration.getAsDouble());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
