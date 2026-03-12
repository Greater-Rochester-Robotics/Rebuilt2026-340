package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.math.geometry.ExtTranslation;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooters;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");

    private static final TunableDouble shootingVelocity = tunables.value("shootingVelocity", 1.25);
    private static final TunableInteger intakeMinRqTagsSeen = tunables.value("intakeMinRqTagsSeen", 15);

    private static final TunableTable grabTunables = tunables.getNested("grabSpeeds");
    private static final TunableDouble turkiyeSpecialGrabSpeed = grabTunables.value("turkiyeSpecial", 2.0);
    private static final TunableDouble secondHelpingFirstGrabSpeed = grabTunables.value("secondHelpingFirst", 2.5);
    private static final TunableDouble secondHelpingSecondGrabSpeed = grabTunables.value("secondHelpingSecond", 2.75);

    private static final TunableTable intakeTunables = tunables.getNested("intake");
    private static final TunableDouble intakeDeceleration = intakeTunables.value("deceleration", 12.0);
    private static final TunableDouble intakeEndTolerance = intakeTunables.value("endTolerance", 0.4);

    private static final TunableTable defaultTunables = tunables.getNested("default");
    private static final TunableDouble velocity = defaultTunables.value("velocity", 4.5);
    private static final TunableDouble deceleration = defaultTunables.value("deceleration", 6.0);
    private static final TunableDouble endTolerance = defaultTunables.value("endTolerance", 0.1);
    private static final TunableDouble endAngTolerance = defaultTunables.value("endAngTolerance", Math.toRadians(6.0));

    private final Hood hood;
    private final Intake intake;
    private final Shooters shooters;
    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        hood = robot.hood;
        intake = robot.intake;
        shooters = robot.shooters;
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.add("Mashup Monday", mashupMonday());
        chooser.add("Turkiye Special", turkiyeSpecial());
        chooser.add("Second Helping Right", secondHelping(false));
        chooser.add("Second Helping Left", secondHelping(true));
        chooser.add("Tower Time Right", towerTime(false));
        chooser.add("Tower Time Left", towerTime(true));
        chooser.add("Simple Sally Right", simpleSally(false));
        chooser.add("Simple Sally Left", simpleSally(true));
    }

    /**
     * The "Mashup Monday" auto routine.
     */
    private Command mashupMonday() {
        return turkiyeSpecial().withTimeout(15.0).andThen(new ScheduleCommand(routines.climb(() -> false, false)));
    }

    /**
     * The "Turkiye Special" auto routine.
     */
    private Command turkiyeSpecial() {
        var shoot = new ExtTranslation(2.875, 2.85);
        var prePickup = new ExtTranslation(1.25, 1.25);
        var pickup = new ExtPose(0.55, 0.61, Rotation2d.k180deg);

        return sequence(
            grab(false, turkiyeSpecialGrabSpeed),
            deadline(apfShooting(prePickup).withTimeout(2.15), routines.shoot().asProxy()),
            deadline(apfDefaultsForever(pickup).withTimeout(2.5), getReady()),
            deadline(apfShooting(shoot), routines.shoot().asProxy())
        );
    }

    /**
     * The "Second Helping" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command secondHelping(boolean left) {
        return sequence(
            sequence(
                grab(left, secondHelpingFirstGrabSpeed),
                parallel(swerve.aimAtTarget(), routines.shoot().asProxy())
            ).withTimeout(10.0),
            grabMore(left, secondHelpingSecondGrabSpeed),
            parallel(swerve.aimAtTarget(), routines.shoot().asProxy())
        );
    }

    /**
     * The "Tower Time" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command towerTime(boolean left) {
        return simpleSally(left).withTimeout(15.0).andThen(new ScheduleCommand(routines.climb(() -> left, false)));
    }

    /**
     * The "Simple Sally" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command simpleSally(boolean left) {
        return sequence(grab(left, () -> 2.0), parallel(swerve.aimAtTarget(), routines.shoot().asProxy()));
    }

    // ********** Helper Methods **********

    /**
     * Sweeps the neutral zone to intake fuel, then returns to a shooting position in the
     * alliance zone while preparing the hood and flywheels. Ends when it reaches that
     * location. Does not run the indexer to actually shoot.
     * @param left {@code true} if the robot should be on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     * @param intakeVelocity The desired cruise velocity of the robot while intaking, in m/s.
     */
    private Command grab(boolean left, DoubleSupplier intakeVelocity) {
        var preIntake = new ExtPose(7.7, 0.9, Rotation2d.fromDegrees(-135.0));
        var preIntakeCrossed = new ExtPose(preIntake.getBlue().getTranslation(), Rotation2d.fromDegrees(95.0));
        var sweep = new ExtPose(7.7, 4.9, Rotation2d.fromDegrees(95.0));
        var preSweep2 = new ExtPose(6.9, 4.9, Rotation2d.fromDegrees(-145.0));
        var sweep2 = new ExtPose(6.25, 2.8, Rotation2d.fromDegrees(-145.0));
        var shoot = new ExtPose(2.875, 2.85, Rotation2d.fromDegrees(-145.0));

        return deadline(
            sequence(
                apfFuelApproach(() ->
                    swerve.inNeutralZone() && swerve.tagsSeen() >= intakeMinRqTagsSeen.get()
                        ? preIntakeCrossed.get(left)
                        : preIntake.get(left)
                ),
                apfIntaking(() -> sweep.get(left), intakeVelocity).withTimeout(4.0),
                swerve.apfDrive(() -> preSweep2.get(left), velocity, () -> 15.0, () -> 0.25, () -> 1e5, false),
                apfIntaking(() -> sweep2.get(left), intakeVelocity).withTimeout(4.0),
                apfDefaults(() -> shoot.get(left))
            ),
            sequence(
                intake.stow().asProxy().withTimeout(1.75),
                intake.intake().asProxy().until(swerve::atAngle),
                getReady()
            )
        );
    }

    /**
     * Sweeps the neutral zone farther in to intake fuel, then returns to a shooting position in the
     * alliance zone while preparing the hood and flywheels. Ends when it reaches that
     * location. Does not run the indexer to actually shoot.
     * @param left {@code true} if the robot should be on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     * @param intakeVelocity The desired cruise velocity of the robot while intaking, in m/s.
     */
    private Command grabMore(boolean left, DoubleSupplier intakeVelocity) {
        var preIntake = new ExtPose(6.7, 2.0, Rotation2d.fromDegrees(-135.0));
        var sweep = new ExtPose(6.7, 5.0, Rotation2d.fromDegrees(95.0));
        var preSweep2 = new ExtPose(6.35, 5.0, Rotation2d.fromDegrees(-120.0));
        var sweep2 = new ExtPose(6.2, 2.8, Rotation2d.fromDegrees(-120.0));
        var shoot = new ExtPose(2.875, 2.85, Rotation2d.fromDegrees(-145.0));

        return deadline(
            sequence(
                apfFuelApproach(() -> preIntake.get(left)),
                apfIntaking(() -> sweep.get(left), intakeVelocity).withTimeout(4.0),
                swerve.apfDrive(() -> preSweep2.get(left), velocity, () -> 15.0, () -> 0.25, () -> 1e5, false),
                apfIntaking(() -> sweep2.get(left), intakeVelocity).withTimeout(4.0),
                apfDefaults(() -> shoot.get(left))
            ),
            sequence(waitSeconds(2.0), intake.intake().asProxy().until(swerve::atAngle), getReady())
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default tolerances of the specified goal location.
     * Uses the default velocity and deceleration values.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfDefaults(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, deceleration, endTolerance, endAngTolerance, false);
    }

    /**
     * Drives the robot to a target position using the P-APF. Uses the
     * default velocity and deceleration values. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfDefaultsForever(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, deceleration, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default intaking tolerances of the specified goal location.
     * Uses the default intaking deceleration rate.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     */
    private Command apfIntaking(Supplier<Pose2d> goal, DoubleSupplier velocity) {
        return swerve.apfDrive(goal, velocity, intakeDeceleration, intakeEndTolerance, () -> 1e5, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, with
     * parameters configured to favorably approach the neutral zone fuel.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfFuelApproach(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, () -> 30.0, () -> 0.5, () -> 1e5, true);
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
