package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
public final class Autos {

    private static final Rotation2d INTAKE_PRE_TAG_ROTATION = Rotation2d.fromDegrees(-145.0);

    private static final TunableTable tunables = Tunables.getNested("autos");

    private static final ExtPose shootPosition = tunables.add(
        "shootPosition",
        new ExtPose(3.2, 2.6, Rotation2d.fromDegrees(-145.0))
    );

    private static final TunableDouble shootingVelocity = tunables.value("shootingVelocity", 1.25);
    private static final TunableInteger intakeMinRqTagsSeen = tunables.value("intakeMinRqTagsSeen", 15);

    private static final TunableTable defaultTunables = tunables.getNested("default");
    private static final TunableDouble velocity = defaultTunables.value("velocity", 4.5);
    private static final TunableDouble deceleration = defaultTunables.value("deceleration", 4.5);
    private static final TunableDouble endTolerance = defaultTunables.value("endTolerance", 0.05);
    private static final TunableDouble endAngTolerance = defaultTunables.value("endAngTolerance", Math.toRadians(6.0));

    private static final TunableTable intakeTunables = tunables.getNested("intake");
    private static final TunableDouble intakeDeceleration = intakeTunables.value("deceleration", 16.0);
    private static final TunableDouble intakeEndTolerance = intakeTunables.value("endTolerance", 0.4);

    private static final TunableTable depotTunables = tunables.getNested("depot");
    private static final TunableDouble depotVelocity = depotTunables.value("velocity", 0.75);
    private static final TunableDouble depotDeceleration = depotTunables.value("deceleration", 4.0);
    private static final TunableDouble depotEndTolerance = depotTunables.value("endTolerance", 0.04);

    private static enum GrabDepth {
        SHALLOW(
            "Shallow",
            new ExtPose(6.7, 2.0, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.7, 4.0, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.35, 4.0, Rotation2d.fromDegrees(-160.0)),
            new ExtPose(6.2, 2.6, Rotation2d.fromDegrees(-95.0))
        ),
        MODERATE(
            "Moderate",
            new ExtPose(7.9, 1.1, Rotation2d.fromDegrees(105.0)),
            new ExtPose(7.9, 3.9, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.7, 3.9, Rotation2d.fromDegrees(-170.0)),
            new ExtPose(6.25, 2.6, Rotation2d.fromDegrees(-90.0))
        ),
        DANGER(
            "Danger",
            new ExtPose(8.4, 1.1, Rotation2d.fromDegrees(105.0)),
            new ExtPose(8.4, 3.85, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.9, 3.85, Rotation2d.fromDegrees(-170.0)),
            new ExtPose(6.25, 2.6, Rotation2d.fromDegrees(-90.0))
        ),
        DOUBLE_DANGER(
            "Double Danger",
            new ExtPose(8.45, 1.1, Rotation2d.fromDegrees(105.0)),
            new ExtPose(8.45, 4.0, Rotation2d.fromDegrees(105.0)),
            new ExtPose(7.9, 4.0, Rotation2d.fromDegrees(-170.0)),
            new ExtPose(7.5, 2.6, Rotation2d.fromDegrees(-90.0))
        ),
        MIDDLE_MANY(
            "Middle Many",
            new ExtPose(7.8, 1.6, Rotation2d.fromDegrees(105.0)),
            new ExtPose(7.8, 5.0, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.3, 5.0, Rotation2d.fromDegrees(-170.0)),
            new ExtPose(6.25, 3.0, Rotation2d.fromDegrees(-90.0))
        ),
        INNIE_OUTIE(
            "Innie Outie",
            new ExtPose(6.2, 2.0, Rotation2d.fromDegrees(95.0)),
            new ExtPose(6.2, 4.7, Rotation2d.fromDegrees(95.0)),
            new ExtPose(7.3, 4.6, Rotation2d.fromDegrees(10.0)),
            new ExtPose(6.9, 3.2, Rotation2d.fromDegrees(-95.0))
        );

        private String name;
        private ExtPose sweepStartPreTags;
        private ExtPose sweepStartPostTags;
        private ExtPose sweepEnd;
        private ExtPose returnStart;
        private ExtPose returnEnd;

        private GrabDepth(String name, ExtPose sweepStart, ExtPose sweepEnd, ExtPose returnStart, ExtPose returnEnd) {
            this.name = name;
            sweepStartPreTags = new ExtPose(sweepStart.getBlue().getTranslation(), INTAKE_PRE_TAG_ROTATION);
            sweepStartPostTags = sweepStart;
            this.sweepEnd = sweepEnd;
            this.returnStart = returnStart;
            this.returnEnd = returnEnd;
        }
    }

    private final Hood hood;
    private final Intake intake;
    private final Shooter shooter;
    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;
    private final AutoChooser firstSwipeChooser;
    private final AutoChooser secondSwipeChooser;

    private boolean doubleSwipeLeft = false;

    public Autos(Robot robot) {
        hood = robot.hood;
        intake = robot.intake;
        shooter = robot.shooter;
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto choosers
        chooser = new AutoChooser("/Autos/chooser");
        firstSwipeChooser = new AutoChooser("/Autos/firstSwipe");
        secondSwipeChooser = new AutoChooser("/Autos/secondSwipe");

        // Disable scheduling for double swipe choosers
        firstSwipeChooser.enableScheduling(false);
        secondSwipeChooser.enableScheduling(false);

        // Add autonomous modes to the dashboard
        chooser.add("Double Swipe Right", doubleSwipe(false));
        chooser.add("Double Swipe Left", doubleSwipe(true));
        chooser.add("Depoo", depoo());
        chooser.add("Simple Sally Right", simpleSally(false));
        chooser.add("Simple Sally Left", simpleSally(true));

        // Add swipe options to the dashboard
        for (var depth : GrabDepth.values()) {
            firstSwipeChooser.add(depth.name, grab(depth, () -> doubleSwipeLeft, false));
            secondSwipeChooser.add(depth.name, grab(depth, () -> doubleSwipeLeft, true));
        }
    }

    private Command doubleSwipe(boolean left) {
        return sequence(
            singleSwipe(firstSwipeChooser::getSelected, left).withTimeout(10.0),
            singleSwipe(secondSwipeChooser::getSelected, left)
        );
    }

    private Command singleSwipe(Supplier<Command> grabCommand, boolean left) {
        return sequence(
            deferredProxy(grabCommand).alongWith(runOnce(() -> doubleSwipeLeft = left)),
            parallel(apfShooting(() -> shootPosition.get(left).getTranslation()), routines.shoot()).asProxy()
        );
    }

    /**
     * The "Depoo" auto routine.
     */
    private Command depoo() {
        var prePickup = new ExtPose(1.25, 5.94, Rotation2d.k180deg);
        var pickup = new ExtPose(0.5, 5.94, Rotation2d.k180deg);

        return sequence(
            grab(GrabDepth.MODERATE, () -> true, false),
            deadline(apfShooting(() -> prePickup.get().getTranslation()).withTimeout(5.0), routines.shoot().asProxy()),
            deadline(apfDepot(pickup).withTimeout(2.0), getReady()),
            deadline(apfDepot(prePickup).withTimeout(2.0), getReady()),
            parallel(apfShooting(() -> shootPosition.get(true).getTranslation()), routines.shoot().asProxy())
        );
    }

    /**
     * The "Simple Sally" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command simpleSally(boolean left) {
        return sequence(
            grab(GrabDepth.MODERATE, () -> left, false),
            parallel(swerve.aimAtTarget(), routines.shoot().asProxy())
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
    private Command grab(GrabDepth depth, BooleanSupplier left, boolean secondPass) {
        return deadline(
            sequence(
                apfFuelApproach(() ->
                    !swerve.inNeutralZone() || swerve.tagsSeen() < intakeMinRqTagsSeen.get()
                        ? depth.sweepStartPreTags.get(left.getAsBoolean())
                        : depth.sweepStartPostTags.get(left.getAsBoolean())
                ),
                apfIntaking(() -> depth.sweepEnd.get(left.getAsBoolean()), secondPass ? 2.25 : 1.6).withTimeout(4.0),
                swerve.apfDrive(
                    () -> depth.returnStart.get(left.getAsBoolean()),
                    velocity,
                    () -> 20.0,
                    () -> 0.25,
                    () -> 1e5,
                    false
                ),
                apfIntaking(() -> depth.returnEnd.get(left.getAsBoolean()), 3.75).withTimeout(3.0),
                apfDefaults(() -> shootPosition.get(left.getAsBoolean()))
            ),
            sequence(waitSeconds(1.5), intake.intake().asProxy().until(swerve::inOurZone), getReady())
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
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default intaking tolerances of the specified goal location.
     * Uses the default intaking deceleration rate.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     */
    private Command apfIntaking(Supplier<Pose2d> goal, double velocity) {
        return swerve.apfDrive(goal, () -> velocity, intakeDeceleration, intakeEndTolerance, () -> 1e5, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default depot tolerances of the specified goal location.
     * Uses the default depot velocity and deceleration alues.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     */
    private Command apfDepot(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, depotVelocity, depotDeceleration, depotEndTolerance, () -> 1e5, false);
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
     * Prepares the hood and shooter to score fuel, and keeps the
     * intake extended. The returned command is already proxied.
     */
    private Command getReady() {
        return parallel(
            intake.intake(),
            hood.targetDistance(swerve::targetDistance, () -> true),
            shooter.targetDistance(swerve::targetDistance, () -> true)
        ).asProxy();
    }
}
