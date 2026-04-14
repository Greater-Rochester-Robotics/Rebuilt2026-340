package org.team340.lib.util.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables.Tunable;

/**
 * A command that does nothing but takes a specified and tunable amount of time to finish.
 *
 */
public class TunableWaitCommand extends Command implements Tunable {

    /** The timer used for waiting. */
    protected Timer timer = new Timer();

    private double duration;

    /**
     * Creates a new TunableWaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param seconds default time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public TunableWaitCommand(double seconds) {
        duration = seconds;
    }

    public void initTunable(TunableTable table) {
        table.value("seconds", duration, v -> duration = v);
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
        return timer.hasElapsed(duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
