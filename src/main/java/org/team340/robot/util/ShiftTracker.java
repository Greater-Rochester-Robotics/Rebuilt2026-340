package org.team340.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableBoolean;
import org.team340.lib.util.Alliance;

/**
 * Tracks hub shift data.
 */
@Logged
public final class ShiftTracker {

    private static final TunableBoolean shiftDefaultWin = Tunables.value("shiftDefaultWin", true);

    private final Timer timer = new Timer();

    public ShiftTracker() {
        RobotModeTriggers.teleop().or(RobotModeTriggers.autonomous()).onTrue(Commands.runOnce(timer::restart));
    }

    /**
     * Returns {@code true} if our alliance's hub is active.
     */
    public boolean active() {
        if (DriverStation.isDisabled()) return false;
        if (DriverStation.isAutonomous()) return true;

        double time = timer.get();
        if (time < 10.0 || time >= 110.0) return true;

        boolean wonAuto = wonAuto();
        if (time >= 85.0) return wonAuto;
        if (time >= 60.0) return !wonAuto;
        if (time >= 35.0) return wonAuto;
        if (time >= 10.0) return !wonAuto;

        return false;
    }

    // Only used for the method below
    private static final double[] SHIFTS = { 0.0, 10.0, 35.0, 60.0, 85.0, 110.0, 140.0 };

    /**
     * Returns the number of seconds left in the current shift period.
     */
    public double shiftTimeLeft() {
        double time = timer.get();

        if (DriverStation.isDisabled()) return 0.0;
        if (DriverStation.isAutonomous()) return 20.0 - time;

        for (int i = SHIFTS.length - 2; i >= 0; i--) {
            if (time >= SHIFTS[i]) return SHIFTS[i + 1] - time;
        }

        return 0.0;
    }

    /**
     * The amount of time left in the current period of the match (auto, teleop).
     */
    public double matchTimeLeft() {
        double time = timer.get();

        if (DriverStation.isDisabled()) return 0.0;
        if (DriverStation.isAutonomous()) return Math.max(20.0 - time, 0.0);
        return Math.max(140.0 - time, 0.0);
    }

    /**
     * Returns {@code true} if our alliance won auto.
     */
    public boolean wonAuto() {
        boolean wonAuto = shiftDefaultWin.get();
        String message = DriverStation.getGameSpecificMessage();
        if (!message.isEmpty()) {
            char c = message.charAt(0);
            if (c == 'B') wonAuto = Alliance.isBlue();
            else if (c == 'R') wonAuto = Alliance.isRed();
        }

        return wonAuto;
    }
}
