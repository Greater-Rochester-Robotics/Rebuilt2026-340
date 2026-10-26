package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.math.Math2;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.AutoChooser;
import org.team340.lib.util.command.VariableWaitCommand;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");

    private static final TunableTable positionsTunables = tunables.getNested("positions");
    private static final ExtPose startingPose = positionsTunables.add(
        "start",
        new ExtPose(4.4, 0.63, Rotation2d.fromDegrees(90.0))
    );

    private static final ExtPose shootPose = positionsTunables.add(
        "shoot",
        new ExtPose(3.0, 2.5, Rotation2d.fromDegrees(-129.0))
    );
    private static final ExtPose preShoot = positionsTunables.add(
        "preShoot",
        new ExtPose(shootPose.getBlue().getTranslation(), Rotation2d.fromDegrees(-90.0))
    );

    private static final ExtPose trenchEnterPose = positionsTunables.add(
        "trenchEnter",
        new ExtPose(3.1, 0.8, Rotation2d.kZero)
    );

    private static final ExtPose transitionExitPose = positionsTunables.add(
        "transitionPrep",
        new ExtPose(6.3, 0.65, Rotation2d.kZero)
    );

    private static final ExtPose transitionFinalPose = positionsTunables.add(
        "transitionFinal",
        new ExtPose(7.3, 1.2, Rotation2d.fromDegrees(25.0))
    );

    private static final TunableDouble swipeTime = tunables.value("swipeTime", 8.8);

    private static final TunableTable defaultTunables = tunables.getNested("default");
    private static final TunableDouble velocity = defaultTunables.value("velocity", 4.5);
    private static final TunableDouble deceleration = defaultTunables.value("deceleration", 4.5);
    private static final TunableDouble endTolerance = defaultTunables.value("endTolerance", 0.06);
    private static final TunableDouble endAngTolerance = defaultTunables.value("endAngTolerance", Math.toRadians(6.0));

    private static final TunableTable intakeTunables = tunables.getNested("intake");
    private static final TunableDouble intakeDeceleration = intakeTunables.value("deceleration", 16.0);
    private static final TunableDouble intakeEndTolerance = intakeTunables.value("endTolerance", 0.4);

    private static final TunableTable depotTunables = tunables.getNested("depot");
    private static final TunableDouble depotVelocity = depotTunables.value("velocity", 0.75);
    private static final TunableDouble depotDeceleration = depotTunables.value("deceleration", 4.0);
    private static final TunableDouble depotEndTolerance = depotTunables.value("endTolerance", 0.04);

    private static final TunableTable trenchingTunables = tunables.getNested("trenching");
    private static final TunableDouble trenchingDeceleration = trenchingTunables.value("deceleration", 13.0);
    private static final TunableDouble trenchingEndTolerance = trenchingTunables.value("endTolerance", 0.4);
    private static final TunableDouble trenchingEndAngTolerance = defaultTunables.value(
        "endAngTolerance",
        Math.toRadians(30.0)
    );

    private static final TunableTable swipeDelays = tunables.getNested("swipeDelays");
    private static final TunableDouble firstSwipeDelay = swipeDelays.value("first", 0.0);
    private static final TunableDouble secondSwipeDelay = swipeDelays.value("second", 0.0);

    private static enum GrabDepth {
        SHALLOW(
            "Shallow",
            new ExtPose(6.7, 1.0, Rotation2d.fromDegrees(90.0)),
            new ExtPose(6.7, 4.0, Rotation2d.fromDegrees(90.0)),
            new ExtPose(6.3, 4.0, Rotation2d.fromDegrees(-160.0)),
            new ExtPose(6.2, 2.6, Rotation2d.fromDegrees(-95.0))
        ),
        MODERATE(
            "Moderate",
            new ExtPose(7.5, 0.9, Rotation2d.fromDegrees(75.0)),
            new ExtPose(7.5, 3.6, Rotation2d.fromDegrees(105.0)),
            new ExtPose(6.2, 3.7, Rotation2d.fromDegrees(170.0)),
            new ExtPose(6.2, 2.7, Rotation2d.fromDegrees(-90.0))
        ),
        DANGER(
            "Danger",
            new ExtPose(7.77, 0.9, Rotation2d.fromDegrees(85.0)),
            new ExtPose(8.27, 3.55, Rotation2d.fromDegrees(110.0)),
            new ExtPose(6.5, 2.9, Rotation2d.fromDegrees(-160.0)),
            new ExtPose(6.2, 2.8, Rotation2d.fromDegrees(-90.0))
        ),
        LITTLE_DANGER(
            "Double Danger",
            new ExtPose(7.85, 1.2, Rotation2d.fromDegrees(85.0)),
            new ExtPose(7.85, 3.3, Rotation2d.fromDegrees(95.0)),
            new ExtPose(7.4, 3.3, Rotation2d.fromDegrees(-170.0)),
            new ExtPose(7.2, 2.15, Rotation2d.fromDegrees(-90.0))
        ),
        INNIE_OUTIE(
            "Innie Outie",
            new ExtPose(5.75, 0.65, Rotation2d.fromDegrees(95.0)),
            new ExtPose(5.65, 3.4, Rotation2d.fromDegrees(85.0)),
            new ExtPose(7.55, 3.8, Rotation2d.fromDegrees(0.0)),
            new ExtPose(7.68, 2.6, Rotation2d.fromDegrees(-92.0))
        );

        private String name;
        private ExtPose sweepStart;
        private ExtPose sweepEnd;
        private ExtPose returnStart;
        private ExtPose returnEnd;

        private GrabDepth(String name, ExtPose sweepStart, ExtPose sweepEnd, ExtPose returnStart, ExtPose returnEnd) {
            this.name = name;
            this.sweepStart = sweepStart;
            this.sweepEnd = sweepEnd;
            this.returnStart = returnStart;
            this.returnEnd = returnEnd;
        }
    }

    private final Hood hood;
    private final Indexer indexer;
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
        indexer = robot.indexer;
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
        chooser.add("Simple Sally Right", simpleSally(false));
        chooser.add("Simple Sally Left", simpleSally(true));

        // Add swipe options to the dashboard
        for (var depth : GrabDepth.values()) {
            firstSwipeChooser.add(depth.name, grab(depth, () -> doubleSwipeLeft, false));
            secondSwipeChooser.add(depth.name, grab(depth, () -> doubleSwipeLeft, true));
        }

        firstSwipeChooser.add("Depoo", depoo());
        secondSwipeChooser.add("Depoo", depoo());
    }

    /**
     * the "Double Swipe" auto routine.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command doubleSwipe(boolean left) {
        return sequence(
            singleSwipe(firstSwipeChooser::getSelected, firstSwipeDelay, left).raceWith(
                new VariableWaitCommand(() -> swipeTime.get() + firstSwipeDelay.get())
            ),
            apfTrenching(() -> trenchEnterPose.get(left)).asProxy(),
            singleSwipe(secondSwipeChooser::getSelected, secondSwipeDelay, left).raceWith(
                new VariableWaitCommand(() -> swipeTime.get() + secondSwipeDelay.get())
            ),
            apfTrenching(() -> trenchEnterPose.get(left)).asProxy(),
            parallel(
                sequence(
                    apfIntaking(() -> transitionExitPose.get(left), velocity),
                    apfIntaking(() -> transitionFinalPose.get(left), velocity)
                ).asProxy(),
                intake.intake().asProxy()
            )
        );
    }

    /**`
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
     * Grabs balls once and shoots them.
     * @param grabCommand A command to grab the balls.
     * @param delay The number of seconds to wait till grabbing balls.
     * @param left {@code true} if the auto should run on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command singleSwipe(Supplier<Command> grabCommand, DoubleSupplier delay, boolean left) {
        Supplier<Command> supplier = () ->
            sequence(
                runOnce(() -> doubleSwipeLeft = left),
                deferredProxy(grabCommand),
                parallel(swerve.aimAtTarget(), routines.shoot()).asProxy()
            );

        return either(
            supplier.get(),
            sequence(new VariableWaitCommand(delay).deadlineFor(swerve.stop(false)).asProxy(), supplier.get()),
            () -> delay.getAsDouble() < 1e-6
        );
    }

    /**
     * Sweeps the neutral zone to intake fuel, then returns to a shooting position in the
     * alliance zone while preparing the hood and flywheels. Ends when it reaches that
     * location. Does not run the indexer to actually shoot.
     * @param left {@code true} if the robot should be on the left side of the field
     *             (from the perspective of the current alliance's driver station),
     *             {@code false} to run on the right side.
     */
    private Command grab(GrabDepth depth, BooleanSupplier left, boolean secondPass) {
        Mutable<Pose2d> preExit = new Mutable<>(Pose2d.kZero);

        return deadline(
            sequence(
                !secondPass ? swerve.resetPose(() -> startingPose.get(left.getAsBoolean())) : none(),
                apfFuelApproach(() ->
                    !swerve.overBump() ? preExit.value : depth.sweepStart.get(left.getAsBoolean())
                ).alongWith(
                    runOnce(() ->
                        preExit.value = new Pose2d(
                            depth.sweepStart.get(left.getAsBoolean()).getTranslation(),
                            new Rotation2d(
                                Math.round(swerve.getPose().getRotation().getRadians() / Math2.HALF_PI) * Math2.HALF_PI
                            )
                        )
                    )
                ),
                apfIntaking(
                    () -> depth.sweepEnd.get(left.getAsBoolean()),
                    () -> secondPass || depth.equals(GrabDepth.DANGER) ? 4.5 : 2.8
                ).withTimeout(3.75),
                swerve
                    .apfDrive(
                        () -> depth.returnStart.get(left.getAsBoolean()),
                        () -> 4.5,
                        () -> 20.0,
                        () -> 0.25,
                        () -> 1e5,
                        false,
                        false
                    )
                    .withTimeout(4.0),
                apfIntaking(() -> depth.returnEnd.get(left.getAsBoolean()), () -> 4.5).withTimeout(5.0),
                apfDefaults(() ->
                    swerve.insideBump() ? shootPose.get(left.getAsBoolean()) : preShoot.get(left.getAsBoolean())
                )
            ),
            sequence(
                waitUntil(swerve::overBump).deadlineFor(
                    either(intake.intake(), intake.stow(), () -> secondPass).asProxy()
                ),
                intake.intake().asProxy().until(swerve::inOurZone),
                getReady()
            )
        );
    }

    /**
     * The "Depoo" auto routine.
     */
    private Command depoo() {
        var prePickup1 = new ExtPose(1.5, 4.8, Rotation2d.fromDegrees(125.0));
        var prePickup2 = new ExtPose(0.585, 4.8, Rotation2d.fromDegrees(100.0));
        var pickup = new ExtPose(0.585, 7.3, Rotation2d.fromDegrees(100.0));

        return sequence(
            deadline(apfDefaults(prePickup1).withTimeout(5.0)),
            deadline(apfDefaults(prePickup2).withTimeout(2.0)),
            deadline(apfDepot(pickup).withTimeout(4.0), getReady()),
            deadline(apfDefaults(() -> shootPose.get(true)), getReady())
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default tolerances of the specified goal location.
     * Uses the default velocity and deceleration values.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfDefaults(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, deceleration, endTolerance, endAngTolerance, false, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default intaking tolerances of the specified goal location.
     * Uses the default intaking deceleration rate.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     */
    private Command apfIntaking(Supplier<Pose2d> goal, DoubleSupplier velocity) {
        return swerve.apfDrive(goal, velocity, intakeDeceleration, intakeEndTolerance, () -> 1e5, false, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default depot tolerances of the specified goal location.
     * Uses the default depot velocity and deceleration values.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfDepot(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, depotVelocity, depotDeceleration, depotEndTolerance, () -> 1e5, false, false);
    }

    /**
     * Drives the robot to a target position using the P-APF, until the robot is
     * positioned within the default trenching tolerances of the specified goal location.
     * Uses the default trenching deceleration rate.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfTrenching(Supplier<Pose2d> goal) {
        return swerve.apfDrive(
            goal,
            velocity,
            trenchingDeceleration,
            trenchingEndTolerance,
            trenchingEndAngTolerance,
            false,
            true
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, with
     * parameters configured to favorably approach the neutral zone fuel.
     * @param goal A supplier that returns the target blue-origin relative field location.
     */
    private Command apfFuelApproach(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, velocity, () -> 30.0, () -> 0.5, () -> 1e5, false, false);
    }

    /**
     * Prepares the hood and shooter to score fuel, and keeps the
     * intake extended. The returned command is already proxied.
     */
    private Command getReady() {
        return parallel(
            indexer.accelerate(),
            intake.intake(),
            hood.targetDistance(swerve::targetDistance, () -> true),
            shooter.targetDistance(swerve::targetDistance, () -> true)
        ).asProxy();
    }
}
