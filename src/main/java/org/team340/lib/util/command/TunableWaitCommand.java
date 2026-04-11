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
    protected Timer m_timer = new Timer();

    private static double m_duration;
    private static String m_defaultName;

    /**
     * Creates a new TunableWaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param seconds default time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public TunableWaitCommand(double seconds) {
        m_duration = seconds;
        m_defaultName = this.getName();
        this.setName(m_defaultName + ": " + m_duration + " seconds");
    }

    public void initTunable(TunableTable table) {
        table.value("seconds", m_duration, v -> {
            m_duration = v;
            this.setName(m_defaultName + ": " + m_duration + " seconds");
        });
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
