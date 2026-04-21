package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions that require
 * multiple subsystems, such as sequences or parallel command groups.
 */
public final class Routines {

    private static final TunableTable tunables = Tunables.getNested("routines");

    private static final TunableDouble staticShootDistance = tunables.value("staticShootDistance", 2.0);
    private static final TunableDouble staticShootHoodPosition = tunables.value("staticShootHoodPosition", 3.0);

    private static final TunableInteger shootingMinRqTagsSeen = tunables.value("shootingMinRqTagsSeen", 15);

    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Lights lights;
    private final Shooter shooter;
    private final Swerve swerve;

    public Routines(Robot robot) {
        hood = robot.hood;
        indexer = robot.indexer;
        intake = robot.intake;
        lights = robot.lights;
        shooter = robot.shooter;
        swerve = robot.swerve;
    }

    /**
     * Barfs fuel out of the intake.
     */
    public Command barf() {
        return parallel(sequence(waitSeconds(0.25), indexer.barf().asProxy()), intake.barf()).withName(
            "Routines.barf()"
        );
    }

    /**
     * Finishes barfing fuel by purging then stowing.
     */
    public Command finishBarf() {
        return sequence(intake.purge().until(intake::isStowed), intake.stow());
    }

    /**
     * Shoots at the hub, without commanding the drivetrain.
     */
    public Command shoot() {
        return shoot(() -> false, () -> false);
    }

    /**
     * Shoots at the current swerve target, without commanding the drivetrain.
     * @param runIntake Whether the intake should also be intaking.
     * @param force A supplier that if {@code true} will force the indexer to feed the shooter.
     */
    public Command shoot(BooleanSupplier runIntake, BooleanSupplier force) {
        return parallel(
            hood.targetDistance(swerve::targetDistance, swerve::inOurZone),
            shooter.targetDistance(swerve::targetDistance, swerve::inOurZone),
            sequence(
                sequence(
                    waitSeconds(0.05),
                    waitUntil(
                        () ->
                            (hood.atPosition()
                                && shooter.atVelocity()
                                && swerve.aimingAtTarget()
                                && swerve.tagsSeen() >= shootingMinRqTagsSeen.get())
                            || force.getAsBoolean()
                    )
                ).deadlineFor(indexer.accelerate()),
                indexer.feed()
            ),
            sequence(
                race(waitUntil(runIntake), waitSeconds(1.0)),
                either(intake.intake().onlyWhile(runIntake), intake.compress().until(runIntake), runIntake)
            ).repeatedly()
        ).withName("Routines.shoot()");
    }

    /**
     * Shoots at the hub from a fixed distance, as a backup.
     */
    public Command staticShoot() {
        return parallel(
            hood.targetDistance(staticShootHoodPosition, () -> true),
            shooter.targetDistance(staticShootDistance, () -> true),
            indexer.feed()
        ).withName("Routines.shoot()");
    }

    /**
     * Shoots at the hub, with driver input and automated heading aim.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param runIntake Whether the intake should also be intaking.
     * @param force A supplier that if {@code true} will force the indexer to feed the .
     */
    public Command driverShoot(DoubleSupplier x, DoubleSupplier y, BooleanSupplier runIntake, BooleanSupplier force) {
        return parallel(shoot(runIntake, force), swerve.aimAtTarget(x, y)).withName("Routines.driverShoot()");
    }

    /**
     * Aim at the hub without running the indexer (to get in last shots).
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driverShootShutdown(DoubleSupplier x, DoubleSupplier y) {
        return deadline(
            waitSeconds(0.4),
            hood.targetDistance(swerve::targetDistance, swerve::inOurZone),
            indexer.clear(),
            shooter.targetDistance(swerve::targetDistance, swerve::inOurZone),
            swerve.aimAtTarget(x, y)
        ).withName("Routines.driverShootShutdown()");
    }

    /**
     * Displays the pre-match lights animation.
     */
    public Command lightsPreMatch() {
        return lights.preMatch(swerve::getPose, swerve::seesAprilTag).withName("Routines.lightsPreMatch()");
    }

    /**
     * Runs all subsystems to check functionality.
     */
    public Command testSequence() {
        Timer timer = new Timer();

        final double SHOOT_TIME = 4.0;
        final double MAX_SHOOT_DISTANCE = 12.0;
        DoubleSupplier targetDistance = () -> timer.get() * (MAX_SHOOT_DISTANCE / SHOOT_TIME);

        return sequence(
            intake.intake().asProxy().withTimeout(2.0),
            parallel(indexer.feed(), intake.compress()).asProxy().withTimeout(2.0),
            parallel(
                intake.stow(),
                hood.targetDistance(targetDistance, () -> true),
                shooter.targetDistance(targetDistance, () -> true)
            )
                .beforeStarting(timer::restart)
                .asProxy()
                .withTimeout(SHOOT_TIME)
        ).withName("Routines.testSequence()");
    }

    /**
     * Runs tuning commands for the shooter and hood.
     */
    public Command tune() {
        return parallel(
            hood.tune(),
            indexer.accelerate().withTimeout(1.0).andThen(indexer.feed()),
            shooter.tune(),
            swerve.aimAtTarget()
        ).withName("Routines.tune()");
    }
}
