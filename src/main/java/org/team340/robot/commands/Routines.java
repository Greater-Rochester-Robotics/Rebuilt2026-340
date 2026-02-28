package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooters;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.Field;
import org.team340.robot.util.Vision.TagMode;

/**
 * The Routines class contains command compositions that require
 * multiple subsystems, such as sequences or parallel command groups.
 */
public final class Routines {

    private static final TunableTable tunables = Tunables.getNested("routines");

    // spotless:off
    private static final TunableDouble climbingMaxDeceleration = tunables.value("climbingMaxDeceleration", 4.0);
    private static final TunableDouble climbingEndTolerance = tunables.value("climbingEndTolerance", 0.01);
    private static final TunableDouble climbingEndAngTolerance = tunables.value("climbingEndAngTolerance", Math.toRadians(1.0));
    // spotless:on

    private final Climber climber;
    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooters shooters;
    private final Swerve swerve;

    public Routines(Robot robot) {
        climber = robot.climber;
        hood = robot.hood;
        indexer = robot.indexer;
        intake = robot.intake;
        shooters = robot.shooters;
        swerve = robot.swerve;
    }

    /**
     * Barfs fuel out of the intake.
     */
    public Command barf() {
        return parallel(indexer.barf(), intake.barf()).withName("Routines.barf()");
    }

    /**
     * Shoots at the hub, without commanding the drivetrain.
     */
    public Command shoot() {
        return parallel(
            hood.targetDistance(swerve::targetDistance),
            shooters.targetDistance(swerve::targetDistance),
            sequence(
                waitSeconds(0.05),
                waitUntil(() -> hood.atPosition() && shooters.atVelocity() && swerve.aimingAtTarget()),
                indexer.feed()
            ),
            sequence(waitSeconds(2.0), intake.stow())
        ).withName("Routines.shoot()");
    }

    /**
     * Shoots at the hub, with driver input and automated heading aim.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driverShoot(DoubleSupplier x, DoubleSupplier y) {
        return parallel(shoot(), swerve.aimAtTarget(x, y)).withName("Routines.driverShoot()");
    }

    /**
     * Aim at the hub without running the indexer (to get in last shots).
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driverShootShutdown(DoubleSupplier x, DoubleSupplier y) {
        return deadline(
            waitSeconds(0.5),
            hood.targetDistance(swerve::targetDistance),
            shooters.targetDistance(swerve::targetDistance),
            swerve.aimAtTarget(x, y)
        ).withName("Routines.driverShootShutdown()");
    }

    /**
     * Auto-drives to the tower and climbs.
     * @param left {@code true} to climb the left tower upright, {@code false} to climb the right upright.
     * @param l3 {@code true} to climb L3, {@code false} to climb L1.
     */
    public Command climb(BooleanSupplier left, boolean l3) {
        Mutable<Boolean> atPosition = new Mutable<>(false);

        return parallel(
            either(climber.climbL3(atPosition::get), climber.climbL1(atPosition::get), () -> l3),
            sequence(
                swerve.apfDrive(
                    () -> left.getAsBoolean() ? Field.TOWER_LEFT_APPROACH.get() : Field.TOWER_RIGHT_APPROACH.get(),
                    climbingMaxDeceleration,
                    climbingEndTolerance,
                    climbingEndAngTolerance
                ),
                swerve.stop(false).withTimeout(0.1),
                swerve.apfDrive(
                    () -> left.getAsBoolean() ? Field.TOWER_LEFT_CLIMB.get() : Field.TOWER_RIGHT_CLIMB.get(),
                    climbingMaxDeceleration,
                    climbingEndTolerance,
                    climbingEndAngTolerance
                ),
                runOnce(() -> atPosition.value = true)
            ),
            swerve.setTagMode(TagMode.TOWER)
        )
            .beforeStarting(() -> atPosition.value = false)
            .withName("Routines.climbL3(" + l3 + ")");
    }
}
