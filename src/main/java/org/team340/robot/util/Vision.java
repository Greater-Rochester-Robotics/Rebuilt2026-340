package org.team340.robot.util;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
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
import org.team340.lib.util.Alliance;

/**
 * Manages all of the robot's cameras.
 */
@Logged
public final class Vision {

    public static final record CameraConfig(String name, Translation3d translation, Rotation3d rotation) {}

    private static final TunableTable tunables = Tunables.getNested("vision");
    private static final TunableDouble zTolerance = tunables.value("zTolerance", 0.5);
    private static final TunableDouble velLatencyFactor = tunables.value("velLatencyFactor", 0.045);
    private static final TunableDouble velAngThreshold = tunables.value("velAngThreshold", 0.8);

    private static enum StrategyWeights {
        MULTITAG(0.14, 0.35),
        TRIG(0.12, 1e5),
        AMBIGUITY(0.2, 0.5);

        public final TunableDouble xy;
        public final TunableDouble angular;

        private StrategyWeights(double xy, double angular) {
            this.xy = tunables.value("xyWeights/" + name(), xy);
            this.angular = tunables.value("angularWeights/" + name(), angular);
        }
    }

    // Set to true to enable simulation. Note that this may
    // negatively impact performance on slower devices.
    private static boolean ENABLE_SIM = false;

    private final Camera[] cameras;
    private final VisionSystemSim sim;

    // Variables for Epilogue logging.
    private final List<Pose3d> estimates = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    private TagMode tagMode = TagMode.ALLIANCE_HUB;

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

        // Enum warmup
        StrategyWeights.MULTITAG.xy.get();
    }

    /**
     * Clears all heading data in the buffer, and adds a new seed. It is
     * recommended to call this method after resetting the robot's pose or rotation.
     * @param rotation Field-relative robot yaw at the given timestamp.
     * @param timestamp Timestamp of the robot yaw data (seconds).
     */
    public void resetHeadingData(Rotation2d rotation, double timestamp) {
        for (var camera : cameras) camera.resetHeadingData(rotation, timestamp);
    }

    /**
     * Gets unread results from all cameras.
     * @param poseHistory Robot pose estimates from the last robot cycle.
     * @param odometryPose The uncorrected odometry pose of the robot. Used for simulation.
     * @param velocity The directionless velocity of the robot in m/s.
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
            camera.refresh(measurements, estimates, targets, velocity);
        }

        // Convert the vision measurement list to an array and return it.
        return measurements.toArray(new VisionMeasurement[measurements.size()]);
    }

    private class Camera {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;
        private final Debouncer enabledDebounce = new Debouncer(5.0, DebounceType.kFalling);

        /**
         * Create a camera.
         * @param cameraName The configured name of the camera.
         * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
         */
        private Camera(CameraConfig config) {
            Transform3d robotToCamera = new Transform3d(config.translation(), config.rotation());

            camera = new PhotonCamera(config.name());
            estimator = new PhotonPoseEstimator(FieldInfo.aprilTags(), robotToCamera);

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
         * Clears all heading data in the buffer, and adds a new seed.
         * @param rotation Field-relative robot yaw at the given timestamp.
         * @param timestamp Timestamp of the robot yaw data (seconds).
         */
        private void resetHeadingData(Rotation2d rotation, double timestamp) {
            estimator.resetHeadingData(timestamp, rotation);
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
         * @see https://github.com/Greater-Rochester-Robotics/Rebuilt2026-340/issues/69
         * @param measurements A list of vision measurements to add to.
         * @param estimates A list of pose estimates to add to for telemetry.
         * @param targets A list of targets to add to for telemetry.
         * @param velocity The directionless velocity of the robot in m/s.
         */
        private void refresh(
            List<VisionMeasurement> measurements,
            List<Pose3d> estimates,
            List<Pose3d> targets,
            double velocity
        ) {
            boolean enabled = enabledDebounce.calculate(DriverStation.isEnabled());

            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                // The usage of EstimatedRobotPose.targetsUsed below leverages our modifications
                // to PhotonPoseEstimator. Utilized strategies have been modified to return
                // PhotonTrackedTarget lists with only the AprilTags used for the solve.

                Optional<EstimatedRobotPose> estimate = Optional.empty();
                double xyWeight = 1e5;
                double angularWeight = 1e5;

                // Falls through until a suitable solve strategy is found.
                do {
                    var multitag = result.getMultiTagResult();

                    if (
                        multitag.isPresent()
                        && !multitag.get().fiducialIDsUsed.isEmpty()
                        && useTag(multitag.get().fiducialIDsUsed.get(0))
                        && (estimate = estimator.estimateCoprocMultiTagPose(result)).isPresent()
                    ) {
                        xyWeight = StrategyWeights.MULTITAG.xy.get();
                        angularWeight = StrategyWeights.MULTITAG.angular.get();
                        break;
                    }

                    if (
                        enabled
                        && result.getBestTarget() != null
                        && useTag(result.getBestTarget().fiducialId)
                        && (estimate = estimator.estimatePnpDistanceTrigSolvePose(result)).isPresent()
                    ) {
                        xyWeight = StrategyWeights.TRIG.xy.get();
                        angularWeight = StrategyWeights.TRIG.angular.get();
                        break;
                    }

                    if (!enabled && (estimate = estimator.estimateLowestAmbiguityPose(result)).isPresent()) {
                        boolean invalidTag = false;
                        for (var target : estimate.get().targetsUsed) {
                            if (!useTag(target.fiducialId)) invalidTag = true;
                        }

                        if (invalidTag) continue;

                        xyWeight = StrategyWeights.AMBIGUITY.xy.get();
                        angularWeight = StrategyWeights.AMBIGUITY.angular.get();
                        break;
                    }
                } while (false);

                // Escape if we couldn't retrieve a suitable estimate.
                if (estimate.isEmpty()) continue;

                // Escape if the estimate has no targets (this shouldn't happen).
                if (estimate.get().targetsUsed.isEmpty()) {
                    DriverStation.reportWarning("Encountered EstimatedRobotPose with no targets", false);
                    continue;
                }

                // Escape if the estimated pose's Z position is outside an acceptable tolerance.
                if (Math.abs(estimate.get().estimatedPose.getZ()) > zTolerance.get()) continue;

                // Calculate the average distance to the detected AprilTags.
                double avgDistance = 0.0;
                for (var target : estimate.get().targetsUsed) {
                    avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }

                avgDistance /= estimate.get().targetsUsed.size();

                // Calculate the pose estimation weights for X/Y location. As
                // distance increases, the tag is trusted exponentially less.
                double xyStd = xyWeight * avgDistance * avgDistance;

                // Calculate the angular pose estimation weight, similar to X/Y.
                double angularStd = angularWeight * avgDistance * avgDistance;

                // Discard our angular pose estimate if the robot's velocity is above an arbitrary threshold.
                if (velocity > velAngThreshold.get()) angularStd = 1e5;

                // Apply a heuristic to the measurement's latency, that accounts for
                // increased translational error at high chassis velocities.
                double timestamp = estimate.get().timestampSeconds - (velocity * velLatencyFactor.get());

                // Push the measurement to the supplied measurements list.
                measurements.add(
                    new VisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        timestamp,
                        VecBuilder.fill(xyStd, xyStd, angularStd)
                    )
                );

                // Push the pose estimate to the estimates list for telemetry.
                estimates.add(estimate.get().estimatedPose);

                // Push the locations of the tags to the targets list for telemetry.
                for (var target : estimate.get().targetsUsed) {
                    var pose = FieldInfo.aprilTags().getTagPose(target.fiducialId);
                    if (pose.isPresent()) targets.add(pose.get());
                    else DriverStation.reportWarning(
                        "Unknown AprilTag ID slipped through: " + target.fiducialId,
                        false
                    );
                }
            }
        }
    }

    /** AprilTag ID filtering mode. */
    public static enum TagMode {
        /** Use the tags on either hub. */
        BOTH_HUBS,
        /** Use only our alliance's hub tags. */
        ALLIANCE_HUB
    }

    /**
     * Sets the tag mode to filter for.
     * @param newMode The new AprilTag ID filtering mode.
     */
    public void setTagMode(TagMode newMode) {
        tagMode = newMode;
    }

    /**
     * Returns {@code true} if an AprilTag should be utilized.
     * @param id The ID of the AprilTag.
     */
    private boolean useTag(int id) {
        boolean invalidAlliance = DriverStation.getAlliance().isEmpty();

        switch (tagMode) {
            case BOTH_HUBS:
            case ALLIANCE_HUB:
                boolean blueHub = (id >= 18 && id <= 21) || (id >= 24 && id <= 27);
                boolean redHub = (id >= 2 && id <= 5) || (id >= 8 && id <= 11);

                if (invalidAlliance || tagMode.equals(TagMode.BOTH_HUBS)) return blueHub || redHub;
                else return Alliance.isBlue() ? blueHub : redHub;
            default:
                return false;
        }
    }
}
