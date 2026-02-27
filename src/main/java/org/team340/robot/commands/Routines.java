package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooters;
import org.team340.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions that require
 * multiple subsystems, such as sequences or parallel command groups.
 */
public final class Routines {

    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooters shooters;
    private final Swerve swerve;

    public Routines(Robot robot) {
        hood = robot.hood;
        indexer = robot.indexer;
        intake = robot.intake;
        shooters = robot.shooters;
        swerve = robot.swerve;
    }

    public Command shoot() {
        return parallel(
            hood.targetDistance(swerve::hubDistance),
            shooters.targetDistance(swerve::hubDistance),
            sequence(parallel(waitUntil(swerve::aimingAtHub), waitSeconds(0.25)), indexer.feed()),
            sequence(waitSeconds(2.0), intake.stow())
        ).withName("Routines.shoot()");
    }

    public Command driverShoot(DoubleSupplier x, DoubleSupplier y) {
        return parallel(shoot(), swerve.aimAtHub(x, y)).withName("Routines.driverShoot()");
    }

    public Command barf() {
        return parallel(indexer.unjam(), intake.barf()).withName("Routines.barf()");
    }
}
