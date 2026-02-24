package org.team340.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.math.Math2;
import org.team340.lib.math.PAPFController;
import org.team340.lib.math.PAPFController.Obstacle;
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
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.util.Field;
import org.team340.robot.util.Vision;
import org.team340.robot.util.Vision.CameraConfig;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double OFFSET = Units.inchesToMeters(12.5);

    private static final TunableTable tunables = Tunables.getNested("swerve");

    private static final TunableDouble aimAtHubTolerance = tunables.value("aimAtHubTolerance", 0.0);
    private static final TunableDouble zeroVelocityTolerance = tunables.value("zeroVelocityTolerance", 0.0);

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FL_ENCODER, 0.0, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FR_ENCODER, 0.0, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BL_ENCODER, 0.0, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BR_ENCODER, 0.0, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(LoggedRobot.DEFAULT_PERIOD, 0.004, 0.02, 0.02)
        .setMovePID(0.25, 0.0, 0.0)
        .setMoveFF(0.0, 0.125)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(true, true)
        .setLimits(5.0, 0.01, 18.0, 15.0, 45.0)
        .setDriverProfile(5.0, 1.5, 0.1, 5.4, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 70.0, 60.0, 60.0)
        .setMechanicalProperties(675.0 / 112.0, 287.0 / 11.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.canandgyro(RobotMap.CANANDGYRO))
        .setPhoenixFeatures(RobotMap.CANBus, true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    // TODO: Populate camera locations.
    private final CameraConfig[] cameras = {};

    @NotLogged
    private final SwerveState state;

    private final SwerveAPI api;
    private final Vision vision;
    private final PAPFController apf;
    private final ProfiledPIDController angularPID;

    private double distanceToHub = 0.0;
    private double angleToHub = 0.0;
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
        api.refresh();

        // Apply vision estimates to the pose estimator.
        final var measurements = vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity);
        seesAprilTag = measurements.length > 0;
        api.addVisionMeasurements(measurements);

        final double deltaX = state.pose.getX() - Field.HUB.get().getX();
        final double deltaY = state.pose.getY() - Field.HUB.get().getY();

        distanceToHub = Math.hypot(deltaX, deltaY);
        angleToHub = Math.atan2(deltaY, deltaX);
    }

    /**
     * Returns {@code true} if an AprilTag has been seen since the last robot loop.
     */
    @NotLogged
    public boolean seesAprilTag() {
        return seesAprilTag;
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.OPERATOR))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> api.resetPose(pose.get()))
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
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration, DoubleSupplier endTolerance) {
        return apfDrive(goal, maxDeceleration)
            .until(() -> Math2.isNear(goal.get().getTranslation(), state.translation, endTolerance.getAsDouble()))
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
     * Drives the robot to a target position using the P-APF while aiming at the hub. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command aimAtHub(final Supplier<Translation2d> goal, final DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.aimAtHub()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                var speeds = apf.calculate(state.pose, goal.get(), config.velocity, maxDeceleration.getAsDouble());

                speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), angleToHub);

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /*
     * Drives the robot to a target position using the P-APF while aiming in the direction of our alliance zone (0 or PI radians). This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command aimAtOurZone(final DoubleSupplier x, final DoubleSupplier y) {
        final double target = Alliance.isBlue() ? Math.PI : 0.0;

        return commandBuilder("Swerve.aimAtOurZone()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                final double angularVelocity = angularPID.calculate(state.rotation.getRadians(), target);

                var speeds = api.calculateDriverSpeeds(distanceToHub, angleToHub, angularVelocity);

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /*
     * Checks if the robot is aiming at the hub and not rotating within a tolerance.
     * @return True if the robot is aiming at the hub, false otherwise.
     */
    public boolean aimingAtHub() {
        final double angleDifference = Math.abs(state.rotation.getRadians() - angleToHub);
        final double angleTolerance = aimAtHubTolerance.get();

        return (
            (angleDifference < angleDifference || angleDifference > Math2.TWO_PI - angleTolerance)
            && Math.abs(state.speeds.omegaRadiansPerSecond) < zeroVelocityTolerance.get()
        );
    }

    /**
     * Checks if the origin of the robot is in our alliance's zone (the blue zone if we are on the blue alliance, and the red zone if we are on the red alliance).
     * @return True if we are in our zone, false otherwise.
     */
    public boolean inOurZone() {
        return Alliance.isBlue() ? Field.BLUE_ZONE > state.pose.getX() : Field.RED_ZONE < state.pose.getX();
    }

    /**
     * Checks if the origin of the robot is in the neutral zone (between the blue zone and the red zone).
     * @return True if we are in the neutral zone, false otherwise.
     */
    public boolean inNeutralZone() {
        final double x = state.pose.getX();
        return Field.BLUE_ZONE <= x && Field.RED_ZONE >= x;
    }

    /**
     * Checks if the origin of the robot is in the opposing alliance's zone (the red zone if we are on the blue alliance, and the blue zone if we are on the red alliance).
     * @return True if we are in the opposing alliance's zone, false otherwise.
     */
    public boolean inTheirZone() {
        return Alliance.isBlue() ? Field.RED_ZONE < state.pose.getX() : Field.BLUE_ZONE > state.pose.getX();
    }

    /**
     * Returns the distance from the origin of our robot to the center of the hub in meters.
     * @return The distance from the origin of our robot to the center of the hub, recalculated every code cycle.
     */
    @NotLogged
    public double distanceToHub() {
        return distanceToHub;
    }

    /**
     * Returns the angle from the origin of our robot to the center of the hub in radians.
     * @return The angle from the origin of our robot to the center of the hub, recalculated every code cycle.
     */
    @NotLogged
    public double angleToHub() {
        return angleToHub;
    }
}
