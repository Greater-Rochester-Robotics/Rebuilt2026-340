package org.team340.robot.subsystems;

import static org.team340.robot.util.ShootParams.TOF;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private static final double SHOOTER_OFFSET = Units.inchesToMeters(-5.75);

    private static final TunableTable tunables = Tunables.getNested("swerve");

    private static final TunableBoolean enableSOTM = tunables.value("enableSOTM", true);
    private static final TunableDouble distanceFudge = tunables.value("distanceFudge", 0.05);
    private static final TunableDouble apfBumpVelocity = tunables.value("apfBumpVelocity", 1.85);
    private static final TunableDouble aimAtHubTolerance = tunables.value("aimAtHubTolerance", Math.toRadians(15.0));
    private static final TunableDouble flatTolerance = tunables.value("flatTolerance", Math.toRadians(5.0));

    // spotless:off
    private static final TunableTable ferryTargets = tunables.getNested("ferryTargets");
    private static final ExtTranslation leftFerryTarget = ferryTargets.add("left", new ExtTranslation(2.5, 6.5));
    private static final ExtTranslation rightFerryTarget = ferryTargets.add("right", new ExtTranslation(leftFerryTarget.getBlue(true)));
    // spotless:on

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
        .setLimits(4.5, 0.01, 16.0, 12.0, 32.0)
        .setDriverProfile(4.5, 1.5, 0.1, 5.4, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 40.0, 60.0, 40.0)
        .setMechanicalProperties(675.0 / 112.0, 287.0 / 11.0, Units.inchesToMeters(3.87))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.canandgyro(RobotMap.CANANDGYRO))
        .setPhoenixFeatures(RobotMap.CANBus, true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    private final CameraConfig[] cameras = {
        new CameraConfig(
            "lumap1",
            new Translation3d(-0.3107, 0.0, 0.2214),
            new Rotation3d(0.0, Math.toRadians(-33.0), Math.PI)
        )
    };

    @NotLogged
    private final SwerveState state;

    private final SwerveAPI api;
    private final Vision vision;
    private final PAPFController apf;
    private final PIDController angularPID;

    private final Subsystem tagModeMutex = new DummySubsystem();

    private Translation2d target = Translation2d.kZero;
    private double targetDistance = 0.0;
    private double targetAngle = 0.0;

    private int tagsSeen = 0;
    private boolean seesAprilTag = false;
    private boolean aimingAtTarget = false;
    private boolean atAngle = false;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(cameras);
        apf = new PAPFController(8.0, 0.25, 0.01, true, Field.OBSTACLES);

        angularPID = new PIDController(6.0, 0.0, 0.0);
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
            vision.setTagMode(inOurZone() ? TagMode.ALLIANCE_HUB : TagMode.BOTH_HUBS);
        }

        // Apply vision estimates to the pose estimator.
        var measurements = Profiler.run("vision.getUnreadResults()", () ->
            vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity)
        );
        Profiler.run("api.addVisionMeasurements()", () -> api.addVisionMeasurements(measurements));
        seesAprilTag = measurements.length > 0;

        // Calculate the robot's displacement from the target.
        target = inOurZone() ? Field.HUB.get() : (isLeftOfCenter() ? leftFerryTarget.get() : rightFerryTarget.get());
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
            // the shooter considering their offset from the robot's center of rotation, we can take the cross product
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
        targetDistance = Math.hypot(deltaX, deltaY) + distanceFudge.get();
        targetAngle = Math.atan2(deltaY, deltaX);

        // Determine if the robot is aiming at the target, using our configured tolerance.
        double dot = Math.cos(targetAngle) * state.rotation.getCos() + Math.sin(targetAngle) * state.rotation.getSin();
        aimingAtTarget = Math.acos(MathUtil.clamp(dot, -1.0, 1.0)) < aimAtHubTolerance.get();

        // Determine if the robot is at an angle (pitch/roll is non-zero).
        if (
            Math.abs(state.pitch.getRadians()) > flatTolerance.get()
            || Math.abs(state.roll.getRadians()) > flatTolerance.get()
        ) {
            atAngle = true;
            tagsSeen = 0;
        } else {
            atAngle = false;
            tagsSeen += measurements.length;
        }

        Profiler.end();
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
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
     * Commands a translational velocity of zero while aiming at the hub.
     */
    public Command aimAtTarget() {
        return aimAtTarget(() -> 0.0, () -> 0.0);
    }

    /**
     * Drives the robot using driver input, while aiming at the hub.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command aimAtTarget(DoubleSupplier x, DoubleSupplier y) {
        return commandBuilder("Swerve.aimAtTarget()").onExecute(() -> {
            var speeds = api.calculateDriverSpeeds(x.getAsDouble(), y.getAsDouble(), 0.0);
            speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), targetAngle);

            api.applySpeeds(speeds, Perspective.OPERATOR, true, true);
        });
    }

    /**
     * Drives the robot to a target position using the P-APF while aiming at the hub. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfAimAtTarget(
        Supplier<Translation2d> goal,
        DoubleSupplier velocity,
        DoubleSupplier maxDeceleration
    ) {
        return apfDrive(
            () -> new Pose2d(goal.get(), new Rotation2d(targetAngle)),
            velocity,
            maxDeceleration,
            false
        ).withName("Swerve.apfAimAtTarget()");
    }

    /**
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     * @param endAngTolerance The tolerance in radians at which to end the command.
     * @param enableFuel If the fuel staged in the neutral zone should be treated as an obstacle.
     */
    public Command apfDrive(
        Supplier<Pose2d> goal,
        DoubleSupplier velocity,
        DoubleSupplier maxDeceleration,
        DoubleSupplier endTolerance,
        DoubleSupplier endAngTolerance,
        boolean enableFuel
    ) {
        return apfDrive(goal, velocity, maxDeceleration, enableFuel)
            .until(() -> {
                var next = goal.get();
                return (
                    Math2.isNear(next.getTranslation(), state.translation, endTolerance.getAsDouble())
                    && Math2.isNear(next.getRotation(), state.rotation, endAngTolerance.getAsDouble())
                );
            })
            .withName("Swerve.apfDrive()");
    }

    /**
     * Drives the robot to a target position using the P-APF. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param velocity The desired cruise velocity of the robot, in m/s.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param enableFuel If the fuel staged in the neutral zone should be treated as an obstacle.
     */
    public Command apfDrive(
        Supplier<Pose2d> goal,
        DoubleSupplier velocity,
        DoubleSupplier maxDeceleration,
        boolean enableFuel
    ) {
        return commandBuilder("Swerve.apfDrive()").onExecute(() -> {
            boolean bumpSpeed = false;
            if (xPositionInBump(state.pose.getX())) {
                bumpSpeed = true;
            } else {
                double t = (Math.abs(state.speeds.vxMetersPerSecond) - apfBumpVelocity.get()) / config.slipAccel;
                if (t >= 0 && xPositionInBump(state.pose.getX() + t * state.speeds.vxMetersPerSecond)) {
                    bumpSpeed = true;
                }
            }

            Pose2d next = goal.get();
            var speeds = apf.calculate(
                state.pose,
                next.getTranslation(),
                bumpSpeed ? apfBumpVelocity.getAsDouble() : velocity.getAsDouble(),
                maxDeceleration.getAsDouble(),
                enableFuel ? Field.FUEL_OBSTACLES : null
            );

            speeds.omegaRadiansPerSecond = angularPID.calculate(
                state.rotation.getRadians(),
                next.getRotation().getRadians()
            );

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

    /**
     * Checks if the origin of the robot is to the left of center field (from the driver's perspective).
     * @return {@code true} if the robot's origin is left of the center, {@code false} otherwise.
     */
    public boolean isLeftOfCenter() {
        return Alliance.isBlue() ^ (state.pose.getY() < Field.Y_CENTER);
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
     * Returns {@code true} if the robot is at an angle (pitch/roll is non-zero).
     */
    @NotLogged
    public boolean atAngle() {
        return atAngle;
    }

    /**
     * Returns the tags seen since the robot was at an angle.
     * This can be used to determined if the robot's odometry is accurate.
     */
    @NotLogged
    public int tagsSeen() {
        return tagsSeen;
    }

    /**
     * A command that when running sets the tag mode to filter for.
     * @param newMode The new AprilTag ID filtering mode.
     */
    public Command setTagMode(TagMode newMode) {
        return tagModeMutex
            .run(() -> vision.setTagMode(newMode))
            .ignoringDisable(true)
            .withName("Swerve.setTagMode(" + newMode.name() + ")");
    }

    /**
     * Checks if the provided x position is in the red or blue bump.
     * @param x The x value to check.
     * @return True if the x value is in the one of the bumps, false otherwise.
     */
    private boolean xPositionInBump(double x) {
        return (x > Field.BLUE_ZONE && x < Field.BLUE_BUMP_FAR) || (x > Field.RED_BUMP_FAR && x < Field.RED_ZONE);
    }
}
