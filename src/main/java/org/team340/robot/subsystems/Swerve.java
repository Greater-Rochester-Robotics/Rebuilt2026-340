package org.team340.robot.subsystems;

import static org.team340.robot.util.ShootParams.TOF;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.logging.Profiler;
import org.team340.lib.math.Math2;
import org.team340.lib.math.PAPFController;
import org.team340.lib.math.PAPFController.Obstacle;
import org.team340.lib.math.geometry.ExtTranslation;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableBoolean;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.command.DummySubsystem;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.util.Field;
import org.team340.robot.util.Vision;
import org.team340.robot.util.Vision.CameraConfig;
import org.team340.robot.util.Vision.TagMode;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double OFFSET = Units.inchesToMeters(10.375);
    private static final double SHOOTER_OFFSET = Units.inchesToMeters(-6.75);

    private static final TunableTable tunables = Tunables.getNested("swerve");

    private static final ExtTranslation leftTarget = Tunables.add("leftTarget", new ExtTranslation(2.5, 6.0));
    private static final ExtTranslation rightTarget = Tunables.add(
        "rightTarget",
        new ExtTranslation(leftTarget.getBlue(true))
    );

    private static final TunableBoolean enableSOTM = tunables.value("enableSOTM", true);
    private static final TunableDouble aimAtHubTolerance = tunables.value("aimAtHubTolerance", Math.toRadians(15.0));

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FL_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FL_ENCODER, 0.076, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FR_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FR_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FR_ENCODER, 0.057, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BL_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BL_ENCODER, -0.105, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BR_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BR_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BR_ENCODER, -0.290, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(LoggedRobot.DEFAULT_PERIOD)
        .setMovePID(0.25, 0.0, 0.0)
        .setMoveFF(0.0, 0.125)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(true, true)
        .setLimits(5.0, 0.01, 18.0, 15.0, 45.0)
        .setDriverProfile(5.0, 1.5, 0.1, 5.4, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 70.0, 60.0, 60.0)
        .setMechanicalProperties(675.0 / 112.0, 287.0 / 11.0, Units.inchesToMeters(3.87))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.canandgyro(RobotMap.CANANDGYRO))
        .setPhoenixFeatures(RobotMap.CANBus, true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    private final CameraConfig[] cameras = {
        new CameraConfig(
            "lumap1",
            new Translation3d(-0.3107, -0.1345, 0.2214),
            new Rotation3d(0.0, Math.toRadians(-33.0), Math.PI)
        )
    };

    @NotLogged
    private final SwerveState state;

    private final SwerveAPI api;
    private final Vision vision;
    private final PAPFController apf;
    private final ProfiledPIDController angularPID;

    private final Subsystem tagModeMutex = new DummySubsystem();

    private Translation2d target = Translation2d.kZero;
    private double targetDistance = 0.0;
    private double targetAngle = 0.0;
    private boolean aimingAtTarget = false;
    private boolean seesAprilTag = false;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(cameras);
        apf = new PAPFController(6.0, 0.25, 0.01, true, new Obstacle[0]);

        angularPID = new ProfiledPIDController(8.0, 0.0, 0.0, new Constraints(10.0, 26.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        state = api.state;

        tunables.add("api", api);
        tunables.add("apf", apf);
        tunables.add("angularPID", angularPID);
    }

    @Override
    public void periodic() {
        Profiler.start("Swerve.periodic()");

        // Refresh the swerve API.
        Profiler.run("api.refresh()", api::refresh);

        // Set our AprilTag mode.
        if (tagModeMutex.getCurrentCommand() == null) {
            vision.setTagMode(inOurZone() ? TagMode.HUB : TagMode.ALL_TAGS);
        }

        // Apply vision estimates to the pose estimator.
        var measurements = Profiler.run("vision.getUnreadResults()", () ->
            vision.getUnreadResults(state.poseHistory, state.odometryPose)
        );
        Profiler.run("api.addVisionMeasurements()", () -> api.addVisionMeasurements(measurements));
        seesAprilTag = measurements.length > 0;

        // Calculate the robot's displacement from the target.
        target = inOurZone() ? Field.HUB.get() : (isLeftOfCenter() ? leftTarget.get() : rightTarget.get());
        double deltaX = state.pose.getX() - target.getX();
        double deltaY = state.pose.getY() - target.getY();

        // If shoot on the move is enabled, perform the necessary adjustments.
        if (enableSOTM.get()) {
            // Get our field-relative chassis speeds.
            var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.speeds, state.rotation);

            // Compensate for translational robot velocity by shifting our target by the product of the robot's
            // velocity and the ball's time of flight. Because we tuned our shot parameters to always produce
            // ball trajectories with a constant time of flight, this is trivial.

            deltaX += fieldSpeeds.vxMetersPerSecond * TOF;
            deltaY += fieldSpeeds.vyMetersPerSecond * TOF;

            // Compensate for angular robot velocity in a similar fashion. To calculate the field-relative velocity of
            // the shooters considering their offset from the robot's center of rotation, we can take the cross product
            // of the following vectors:
            //
            // [   0   ]   [ offset * rotation.cos ]
            // |   0   | x | offset * rotation.sin |
            // [ omega ]   [           0           ]
            //
            // The first and second element of the resulting vector is the shooter's field-relative X and Y velocity,
            // respectively, caused by the robot's angular velocity. Multiplying each component by the ball's time of
            // flight will produce the desired deltaX and deltaY adjustments.

            deltaX -= state.rotation.getSin() * fieldSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET * TOF;
            deltaY += state.rotation.getCos() * fieldSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET * TOF;
        }

        // Save our target distance and angle.
        targetDistance = Math.hypot(deltaX, deltaY);
        targetAngle = Math.atan2(deltaY, deltaX);

        // Determine if the robot is aiming at the target, using our configured tolerance.
        double dot = Math.cos(targetAngle) * state.rotation.getCos() + Math.sin(targetAngle) * state.rotation.getSin();
        aimingAtTarget = Math.acos(MathUtil.clamp(dot, -1.0, 1.0)) < aimAtHubTolerance.get();

        Profiler.end();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> {
                api.tareRotation(Perspective.OPERATOR);
                vision.resetHeadingData(state.rotation, Timer.getFPGATimestamp());
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> {
                api.resetPose(pose.get());
                vision.resetHeadingData(state.rotation, Timer.getFPGATimestamp());
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.OPERATOR,
                true,
                true
            )
        );
    }

    /**
     * Drives the robot using driver input, while aiming at the hub.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command aimAtTarget(DoubleSupplier x, DoubleSupplier y) {
        return commandBuilder("Swerve.aimAtTarget()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                var speeds = api.calculateDriverSpeeds(x.getAsDouble(), y.getAsDouble(), 0.0);
                speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), targetAngle);

                api.applySpeeds(speeds, Perspective.OPERATOR, true, true);
            });
    }

    /**
     * Drives the robot using driver input, while aiming at our alliance zone.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command aimAtOurZone(DoubleSupplier x, DoubleSupplier y) {
        return commandBuilder("Swerve.aimAtOurZone()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                var speeds = api.calculateDriverSpeeds(x.getAsDouble(), y.getAsDouble(), 0.0);
                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    Alliance.isBlue() ? Math.PI : 0.0
                );

                api.applySpeeds(speeds, Perspective.OPERATOR, true, true);
            });
    }

    /**
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     * @param endAngTolerance The tolerance in radians at which to end the command.
     */
    public Command apfDrive(
        Supplier<Pose2d> goal,
        DoubleSupplier maxDeceleration,
        DoubleSupplier endTolerance,
        DoubleSupplier endAngTolerance
    ) {
        return apfDrive(goal, maxDeceleration)
            .until(
                () ->
                    Math2.isNear(goal.get().getTranslation(), state.translation, endTolerance.getAsDouble())
                    && Math2.isNear(goal.get().getRotation(), state.rotation, endAngTolerance.getAsDouble())
            )
            .withName("Swerve.apfDrive()");
    }

    /**
     * Drives the robot to a target position using the P-APF. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d next = goal.get();
                var speeds = apf.calculate(
                    state.pose,
                    next.getTranslation(),
                    config.velocity,
                    maxDeceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    next.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the robot to a target position using the P-APF while aiming at the
     * hub, until the robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfAimAtHub(
        Supplier<Translation2d> goal,
        DoubleSupplier maxDeceleration,
        DoubleSupplier endTolerance
    ) {
        return apfAimAtTarget(goal, maxDeceleration)
            .until(() -> Math2.isNear(goal.get(), state.translation, endTolerance.getAsDouble()))
            .withName("Swerve.apfAimAtHub()");
    }

    /**
     * Drives the robot to a target position using the P-APF while aiming at the hub. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfAimAtTarget(Supplier<Translation2d> goal, DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.apfAimAtTarget()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                var speeds = apf.calculate(state.pose, goal.get(), config.velocity, maxDeceleration.getAsDouble());
                speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), targetAngle);

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving. This command does not end.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Checks if the origin of the robot is in our alliance's zone (the blue zone if
     * we are on the blue alliance, and the red zone if we are on the red alliance).
     * @return {@code true} if we are in our zone, {@code false} otherwise.
     */
    public boolean inOurZone() {
        return Alliance.isBlue() ? Field.BLUE_ZONE > state.pose.getX() : Field.RED_ZONE < state.pose.getX();
    }

    /**
     * Checks if the origin of the robot is in the neutral zone (between the blue zone and the red zone).
     * @return {@code true} if we are in the neutral zone, {@code false} otherwise.
     */
    public boolean inNeutralZone() {
        final double x = state.pose.getX();
        return Field.BLUE_ZONE <= x && Field.RED_ZONE >= x;
    }

    /**
     * Checks if the origin of the robot is in the opposing alliance's zone (the red zone
     * if we are on the blue alliance, and the blue zone if we are on the red alliance).
     * @return {@code true} if we are in the opposing alliance's zone, {@code false} otherwise.
     */
    public boolean inTheirZone() {
        return Alliance.isBlue() ? Field.RED_ZONE < state.pose.getX() : Field.BLUE_ZONE > state.pose.getX();
    }

    public boolean isLeftOfCenter() {
        return Alliance.isBlue() ^ (state.pose.getY() < Field.Y_CENTER);
    }

    public boolean isLeftOfTower() {
        return Alliance.isBlue() ? state.pose.getY() > Field.BLUE_TOWER_Y : state.pose.getY() < Field.RED_TOWER_Y;
    }

    public boolean isAwayFromTower() {
        return Alliance.isBlue()
            ? state.pose.getX() > Field.TOWER_LEFT_APPROACH.getBlue().getX()
            : state.pose.getX() < Field.TOWER_LEFT_APPROACH.getRed().getX();
    }

    /**
     * Returns the distance from the origin of our robot to the center of the hub in meters.
     */
    @NotLogged
    public double targetDistance() {
        return targetDistance;
    }

    /**
     * Returns the angle from the origin of our robot to the center of the hub in radians.
     */
    @NotLogged
    public double targetAngle() {
        return targetAngle;
    }

    /**
     * Returns {@code true} if the robot is aiming at the hub.
     */
    @NotLogged
    public boolean aimingAtTarget() {
        return aimingAtTarget;
    }

    /**
     * Returns {@code true} if an AprilTag has been seen since the last robot loop.
     */
    @NotLogged
    public boolean seesAprilTag() {
        return seesAprilTag;
    }

    /**
     * A command that when running sets the tag mode to filter for.
     * @param newMode The new AprilTag ID filtering mode.
     */
    public Command setTagMode(TagMode newMode) {
        return tagModeMutex.run(() -> vision.setTagMode(newMode));
    }
}
