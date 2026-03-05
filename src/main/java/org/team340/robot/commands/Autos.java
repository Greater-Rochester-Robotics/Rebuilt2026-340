package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

    private static final TunableDouble deceleration = tunables.value("deceleration", 6.0);
    private static final TunableDouble tolerance = tunables.value("tolerance", 0.1);
    private static final TunableDouble angTolerance = tunables.value("angTolerance", Math.toRadians(6.0));

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
    }

    private Command turkiyeSpecial() {
        var shoot = new ExtTranslation(2.875, 2.85);
        var prePickup = new ExtTranslation(0.75, 1.5);
        var pickup = new ExtPose(0.5, 0.67, Rotation2d.k180deg);

        return sequence(
            grab(false),
            deadline(apfShooting(shoot).withTimeout(0.5), routines.shoot().asProxy()),
            deadline(apfShooting(prePickup).withTimeout(3.0), routines.shoot().asProxy()),
            deadline(swerve.apfDrive(pickup, deceleration).withTimeout(2.5), getReady()),
            deadline(apfShooting(shoot), routines.shoot().asProxy())
        );
    }

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
                    swerve.apfDrive(() -> sweep2.get(left), deceleration, () -> 0.75, angTolerance)
                ),
                intake.intake().asProxy()
            ),
            deadline(apfDefaults(() -> shoot.get(left)), getReady())
        );
    }

    private Command apfDefaults(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, deceleration, tolerance, angTolerance);
    }

    private Command apfIntaking(Supplier<Pose2d> goal) {
        return swerve.apfDrive(goal, intakeVelocity, deceleration, tolerance, angTolerance);
    }

    private Command apfShooting(Supplier<Translation2d> goal) {
        return swerve.apfAimAtTarget(goal, shootingVelocity, deceleration);
    }

    private Command getReady() {
        return parallel(
            intake.extend(),
            hood.targetDistance(swerve::targetDistance),
            shooters.targetDistance(swerve::targetDistance)
        ).asProxy();
    }
}
