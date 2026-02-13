package org.team340.robot.util;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team340.lib.math.FieldInfo;
import org.team340.lib.math.geometry.TimestampedPose;
import org.team340.lib.math.geometry.VisionMeasurement;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.robot.util.PhotonPoseEstimator.ConstrainedSolvepnpParams;

/**
 * Manages all of the robot's cameras.
 */
@Logged
public final class Vision {

    public static final record CameraConfig(String name, Translation3d translation, Rotation3d rotation) {}

    private static final TunableTable tunables = Tunables.getNested("vision");

    private static final TunableDouble trigXyStd = tunables.value("trigXyStd", 0.18);
    private static final TunableDouble constrainedPnpXyStd = tunables.value("constrainedPnpXyStd", 0.4);
    private static final TunableDouble constrainedPnpAngStd = tunables.value("constrainedPnpAngStd", 0.14);
    private static final TunableDouble velocityLatencyFactor = tunables.value("velocityLatencyFactor", 0.06);

    // Set to true to enable simulation. Note that this may
    // negatively impact performance on slower devices.
    private static boolean ENABLE_SIM = false;

    private final Camera[] cameras;
    private final VisionSystemSim sim;

    // Variables for Epilogue logging.
    private final List<Pose2d> estimates = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    /**
     * Create the vision manager.
     * @param cameras Configurations for the robot's cameras.
     */
    public Vision(CameraConfig[] cameras) {
        if (RobotBase.isSimulation() && ENABLE_SIM) {
            sim = new VisionSystemSim("simulation");
            sim.addAprilTags(FieldInfo.aprilTags());
        } else {
            sim = null;
        }

        cameras = RobotBase.isSimulation() && !ENABLE_SIM ? new CameraConfig[0] : cameras;
        this.cameras = Arrays.stream(cameras).map(Camera::new).toArray(Camera[]::new);

        // Hit the undocumented Photon Turbo Button™
        // https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Resets cached measurements utilized by the pose estimators for seeding. It is
     * recommended to call this method after resetting the robot's pose or rotation.
     */
    public void reset() {
        for (var camera : cameras) camera.clearHeadingData();
    }

    /**
     * Gets unread results from all cameras.
     * @param poseHistory Robot pose estimates from the last robot cycle.
     * @param odometryPose The uncorrected odometry pose of the robot. Used for simulation.
     * @param velocity The directionless measured velocity of the robot in m/s.
     */
    public VisionMeasurement[] getUnreadResults(
        List<TimestampedPose> poseHistory,
        Pose2d odometryPose,
        double velocity
    ) {
        if (sim != null) {
            // Update sim, if applicable.
            sim.update(odometryPose);
        }

        // Clear lists from the last cycle.
        // These are only used for telemetry.
        estimates.clear();
        targets.clear();

        // Create a new list to save this cycle's vision measurements to.
        List<VisionMeasurement> measurements = new ArrayList<>();

        // Iterate over every camera.
        for (var camera : cameras) {
            // Update the camera's PhotonPoseEstimator with the robot's pose history. Since
            // we run odometry on a separate thread at a higher frequency, we have multiple
            // timestamped poses to reference which is helpful for latency compensation.
            camera.addReferencePoses(poseHistory);

            // Populate our lists with vision data from the camera.
            camera.refresh(velocity, measurements, targets);
        }

        // Save the Pose2ds from all vision measurements
        // to the estimates list to be logged by Epilogue.
        estimates.addAll(
            measurements
                .stream()
                .map(m -> m.pose())
                .toList()
        );

        // Convert the vision measurement list to an array and return it.
        return measurements.stream().toArray(VisionMeasurement[]::new);
    }

    private class Camera {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;
        private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;
        private final Debouncer disabledDebounce = new Debouncer(5.0, DebounceType.kFalling);

        /**
         * Create a camera.
         * @param cameraName The configured name of the camera.
         * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
         */
        private Camera(CameraConfig config) {
            Transform3d robotToCamera = new Transform3d(config.translation(), config.rotation());

            camera = new PhotonCamera(config.name());
            estimator = new PhotonPoseEstimator(FieldInfo.aprilTags(), PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
            constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));

            if (sim != null) {
                // (Roughly) the properties of an OV9281
                var props = new SimCameraProperties();
                props.setCalibration(1280, 800, Rotation2d.fromDegrees(87.0));
                props.setCalibError(0.4, 0.05);
                props.setAvgLatencyMs(18.0);
                props.setLatencyStdDevMs(10.0);
                props.setFPS(60.0);

                sim.addCamera(new PhotonCameraSim(camera, props), robotToCamera);
            }
        }

        /**
         * Clears all heading data in the buffer.
         */
        private void clearHeadingData() {
            estimator.clearHeadingData();
        }

        /**
         * Adds reference poses to be utilized by the Photon pose estimator.
         * @param poseHistory Robot pose estimates from the last robot cycle.
         */
        private void addReferencePoses(List<TimestampedPose> poseHistory) {
            for (var pose : poseHistory) {
                estimator.addHeadingData(pose.timestamp(), pose.pose().getRotation());
            }
        }

        /**
         * Refreshes the provided lists with new unread results from the camera. Note
         * that this method does not remove any elements from the supplied lists.
         * @param velocity The directionless measured velocity of the robot in m/s.
         * @param measurements A list of vision measurements to add to.
         * @param targets A list of targets to add to.
         */
        private void refresh(double velocity, List<VisionMeasurement> measurements, List<Pose3d> targets) {
            // If we are disabled, use Constrained SolvePNP to estimate the robot's heading.
            // See https://github.com/Greater-Rochester-Robotics/Reefscape2025-340/issues/19
            boolean usingTrig = disabledDebounce.calculate(DriverStation.isEnabled());
            estimator.setPrimaryStrategy(usingTrig ? PNP_DISTANCE_TRIG_SOLVE : CONSTRAINED_SOLVEPNP);

            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                // Get an estimate from the PhotonPoseEstimator.
                var estimate = estimator.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                // Get the target AprilTag, and reject the measurement if the
                // tag is not configured to be utilized by the pose estimator.
                var target = estimate.get().targetsUsed.get(0);
                int id = target.fiducialId;
                if (!useTag(id)) continue;

                // Get the location of the tag on the field.
                var tagLocation = FieldInfo.aprilTags().getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                // Determine the distance from the camera to the tag.
                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                // Calculate the pose estimation weights for X/Y location. As
                // distance increases, the tag is trusted exponentially less.
                double xyStd = (usingTrig ? trigXyStd.get() : constrainedPnpXyStd.get()) * distance * distance;

                // Calculate the angular pose estimation weight. If we're solving via trig, reject
                // the heading estimate to ensure the pose estimator doesn't "poison" itself with
                // essentially duplicate data. Otherwise, weight the estimate similar to X/Y.
                double angStd = (usingTrig ? 1e5 : constrainedPnpAngStd.get()) * distance * distance;

                // Apply a heuristic to the measurement's latency, that accounts for
                // increased translational error at high chassis velocities.
                double timestamp = estimate.get().timestampSeconds - (velocity * velocityLatencyFactor.get());

                // Push the measurement to the supplied measurements list.
                measurements.add(
                    new VisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        timestamp,
                        VecBuilder.fill(xyStd, xyStd, angStd)
                    )
                );

                // Push the location of the tag to the targets list for telemetry.
                targets.add(tagLocation.get());
            }
        }

        /**
         * Returns {@code true} if an AprilTag should be utilized.
         * @param id The ID of the AprilTag.
         */
        private boolean useTag(int id) {
            // TODO: Add tags that we are targeting.
            return false;
        }
    }
}
