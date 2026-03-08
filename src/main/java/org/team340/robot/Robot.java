package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.logging.Profiler;
import org.team340.lib.util.DisableWatchdog;
import org.team340.lib.util.command.RumbleCommand;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Hood;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooters;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.ShiftTracker;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Climber climber;
    public final Hood hood;
    public final Indexer indexer;
    public final Intake intake;
    public final Shooters shooters;
    public final Swerve swerve;

    public final Routines routines;
    public final Autos autos;

    public final ShiftTracker shiftTracker;

    private final CommandXboxController driver;

    public Robot() {
        PhoenixUtil.disableDaemons();

        // Initialize subsystems
        climber = new Climber();
        hood = new Hood();
        indexer = new Indexer();
        intake = new Intake();
        shooters = new Shooters();
        swerve = new Swerve();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize helpers
        shiftTracker = new ShiftTracker();

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);

        // Set default commands
        climber.setDefaultCommand(sequence(waitUntil(swerve::isAwayFromTower), climber.retract(), idle()));
        intake.setDefaultCommand(intake.stow());
        hood.setDefaultCommand(hood.goToZero(false));
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Create triggers
        var shoot = driver.leftBumper().or(driver.rightBumper());
        RobotModeTriggers.teleop().onTrue(climber.unclimbL1());
        new Trigger(() -> shiftTracker.shiftTimeLeft() < 5.0)
            .onTrue(new RumbleCommand(driver, 1.0).withTimeout(0.3).onlyIf(this::isTeleop))
            .onFalse(new RumbleCommand(driver, 1.0).withTimeout(0.6).onlyIf(this::isTeleop));

        // Driver bindings
        driver.a().and(shoot.negate()).onTrue(intake.intake(driver.a()));
        driver.b().whileTrue(routines.barf());
        driver.x().whileTrue(routines.staticShoot());
        driver.y().onTrue(none()); // Reserved for shoot override

        shoot
            .onTrue(routines.driverShoot(this::driverX, this::driverY, driver.a(), driver.y()))
            .onFalse(routines.driverShootShutdown(this::driverX, this::driverY));

        driver.povLeft().onTrue(swerve.tareRotation());
        driver.povRight().whileTrue(climber.zero());
        driver.povUp().whileTrue(routines.climb(swerve::isLeftOfTower, true));
        driver.povDown().whileTrue(hood.goToZero(true));

        driver.rightStick().whileTrue(climber.testServo()); // TODO

        // Disable loop overrun warnings from the command
        // scheduler, since we already log loop timings
        DisableWatchdog.in(scheduler, "m_watchdog");

        // Configure the brownout threshold to match RIO 1
        RobotController.setBrownoutVoltage(6.3);

        // Enable real-time thread priority
        enableRT(true);
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
    }
}
