package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.function.Supplier;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.math.geometry.ExtTranslation;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooters;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");

    public static final TunableDouble intakeVelocity = tunables.value("intakeVelocity", 1.75);
    public static final TunableDouble shootingVelocity = tunables.value("shootingVelocity", 1.0);

    private static final TunableDouble velocity = tunables.value("velocity", 4.5);
    private static final TunableDouble deceleration = tunables.value("deceleration", 6.0);
    private static final TunableDouble endTolerance = tunables.value("endTolerance", 0.1);
    private static final TunableDouble endAngTolerance = tunables.value("endAngTolerance", Math.toRadians(6.0));

    private final Climber climber;
    private final Hood hood;
    private final Intake intake;
    private final Shooters shooters;
    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        climber = robot.climber;
        hood = robot.hood;
        intake = robot.intake;
        shooters = robot.shooters;
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.add("Turkiye Special", turkiyeSpecial());
        chooser.add("Tower Time Right", towerTime(false));
        chooser.add("Tower Time Left", towerTime(true));
    }

    /**
     * The "Turkiye Special" auto routine.
     */
    private Command turkiyeSpecial() {
        var shoot = new ExtTranslation(2.875, 2.85);
        var prePickup = new ExtTranslation(0.75, 1.5);
        var pickup = new ExtPose(0.5, 0.67, Rotation2d.k180deg);

        return sequence(
            grab(false),
            deadline(swerve.aimAtTarget().withTimeout(0.5), routines.shoot().asProxy()),
            deadline(apfShooting(prePickup).withTimeout(3.0), routines.shoot().asProxy()),
            deadline(swerve.apfDrive(pickup, velocity, deceleration).withTimeout(2.5), getReady()),
            deadline(apfShooting(shoot), routines.shoot().asProxy())
        );
    }

    /**
     * The "Tower Time" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command towerTime(boolean left) {
        return sequence(
            deadline(
                waitSeconds(15.0),
                sequence(grab(left), parallel(swerve.aimAtTarget(), routines.shoot().asProxy()))
            ),
            new ScheduleCommand(routines.climb(() -> left, false))
        );
    }

    // ********** Helper Methods **********

    /**
     * Sweeps the neutral zone to intake fuel, then returns to a shooting position in the
     * alliance zone while preparing the hood and flywheels. Ends when it reaches that
     * location. Does not run the indexer to actually shoot.
     * @param left {@code true} if the robot should be on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command grab(boolean left) {
        var preIntake = new ExtPose(7.7, 1.8, Rotation2d.fromDegrees(-135.0));
        var sweep = new ExtPose(7.7, 5.75, Rotation2d.fromDegrees(90.0));
        var sweep2 = new ExtPose(6.4, 2.4, Rotation2d.fromDegrees(-135.0));
        var shoot = new ExtPose(2.875, 2.85, Rotation2d.fromDegrees(-145.0));

        return sequence(
            apfDefaults(() -> preIntake.get(left)),
            deadline(
                sequence(
                    apfIntaking(() -> sweep.get(left)),
                    swerve.apfDrive(() -> sweep2.get(left), velocity, deceleration, () -> 0.75, endAngTolerance)
                ),
                intake.intake().asProxy()
            ),
            deadline(apfDefaults(() -> shoot.get(left)), getReady())
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default tolerances of the specified goal location.
     * Uses the default velocity and deceleration values.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfDefaults(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, deceleration, endTolerance, endAngTolerance);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default tolerances of the specified goal location.
     * Uses {@code intakeVelocity} and the default deceleration rate.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfIntaking(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, intakeVelocity, deceleration, endTolerance, endAngTolerance);
    }

    /**
     * Drives the robot to a target position using the P-APF while aiming at
     * the hub. Uses {@code shootingVelocity} and the default deceleration
     * rate. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfShooting(Supplier<Translation2d> goal) {
        return swerve.apfAimAtTarget(goal, shootingVelocity, deceleration);
    }

    /**
     * Prepares the hood and shooters to score fuel, and keeps the
     * intake extended. The returned command is already proxied.
     */
    private Command getReady() {
        return parallel(
            intake.extend(),
            hood.targetDistance(swerve::targetDistance),
            shooters.targetDistance(swerve::targetDistance)
        ).asProxy();
    }
}
