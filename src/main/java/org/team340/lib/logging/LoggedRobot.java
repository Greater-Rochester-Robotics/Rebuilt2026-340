package org.team340.lib.logging;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import java.util.function.Consumer;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.Tunable;
import org.team340.lib.util.DisableWatchdog;

/**
 * LoggedRobot implements the IterativeRobotBase robot program framework,
 * and is intended to be subclassed by a user creating a robot program.
 *
 * <p>The LoggedRobot class configures the program for logging data to NetworkTables
 * via Epilogue, which is captured by the {@link DataLogManager} alongside Driver
 * Station values and saved to file. Loop timings are also recorded using the
 * {@link Profiler} class.
 *
 * <p>{@code periodic()} functions from the base class are called on an interval
 * by a Notifier instance. Additionally, utility classes with periodic methods
 * such as {@link Tunable} are updated on the same interval.
 */
public class LoggedRobot extends IterativeRobotBase {

    /** Default loop period. */
    public static final double DEFAULT_PERIOD = 0.02;

    private final int notifier = NotifierJNI.initializeNotifier();
    private final Consumer<EpilogueBackend> logger;

    private boolean useRT = false;
    private boolean isRT = false;
    private long nextCycle = 0L;

    /**
     * Constructor for LoggedRobot.
     */
    protected LoggedRobot() {
        this(DEFAULT_PERIOD);
    }

    /**
     * Constructor for LoggedRobot.
     * @param period Period in seconds.
     */
    protected LoggedRobot(double period) {
        super(period);
        NotifierJNI.setNotifierName(notifier, "LoggedRobot");

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DriverStation.silenceJoystickConnectionWarning(true);

        EpilogueProxy.getConfig().root = "/Telemetry";
        logger = EpilogueProxy.getLogger(this);

        DisableWatchdog.in(this, "m_watchdog");
    }

    /**
     * Enables real-time thread priority on the main thread while the robot is enabled.
     * <b>Do not use this method if you are unaware of its side effects.</b>
     * @param enabled {@code true} to enable real-time priority.
     */
    public void enableRT(boolean enabled) {
        useRT = enabled;
    }

    @Override
    public void startCompetition() {
        robotInit();
        if (isSimulation()) {
            simulationInit();
        }

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();
        Profiler.observeProgramReady();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            long now = RobotController.getFPGATime();
            if (nextCycle < now) {
                Profiler.observeOverrun();
                nextCycle = now;
            } else {
                NotifierJNI.updateNotifierAlarm(notifier, nextCycle);
                if (NotifierJNI.waitForNotifierAlarm(notifier) == 0L) {
                    break;
                }
            }

            nextCycle += (long) (getPeriod() * 1000000.0);

            Profiler.start("robot");

            Profiler.run("loopFunction", () -> loopFunc());
            Profiler.run("epilogue", () -> logger.accept(EpilogueProxy.getRootBackend()));
            Profiler.run("tunables", Tunables::update);

            if (isRT != (isRT = useRT && isEnabled())) {
                Threads.setCurrentThreadPriority(isRT, isRT ? 1 : 0);
            }

            Profiler.end();
        }
    }

    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(notifier);
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(notifier);
        NotifierJNI.cleanNotifier(notifier);
        super.close();
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
