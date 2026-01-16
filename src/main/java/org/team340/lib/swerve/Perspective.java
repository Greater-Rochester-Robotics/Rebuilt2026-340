package org.team340.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.team340.lib.util.Alliance;

/**
 * Describes a forward (X+ direction) perspective for {@link ChassisSpeeds}.
 */
public enum Perspective {
    /**
     * The speeds are relative to the operator's perspective. If the robot
     * is on the blue alliance, X+ drives towards the red alliance. If the
     * robot is on the red alliance, X+ drives towards the blue alliance.
     */
    OPERATOR {
        @Override
        public ChassisSpeeds toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return (Alliance.isBlue() ? BLUE : RED).toRobotSpeeds(speeds, robotAngle);
        }

        @Override
        public ChassisSpeeds toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return (Alliance.isBlue() ? BLUE : RED).toPerspectiveSpeeds(speeds, robotAngle);
        }

        @Override
        public Rotation2d getTareRotation() {
            return (Alliance.isBlue() ? BLUE : RED).getTareRotation();
        }
    },

    /**
     * The speeds are relative to the blue alliance perspective.
     * X+ drives towards the red alliance.
     */
    BLUE {
        @Override
        public ChassisSpeeds toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle);
        }

        @Override
        public ChassisSpeeds toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotAngle);
        }

        @Override
        public Rotation2d getTareRotation() {
            return Rotation2d.kZero;
        }
    },

    /**
     * The speeds are relative to the red alliance perspective.
     * X+ drives towards the blue alliance.
     */
    RED {
        @Override
        public ChassisSpeeds toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle.rotateBy(Rotation2d.kPi));
        }

        @Override
        public ChassisSpeeds toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotAngle.rotateBy(Rotation2d.kPi));
        }

        @Override
        public Rotation2d getTareRotation() {
            return Rotation2d.kPi;
        }
    },

    /**
     * The speeds are relative to the robot's perspective.
     * X+ drives forwards relative to the chassis.
     */
    ROBOT {
        @Override
        public ChassisSpeeds toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return speeds;
        }

        @Override
        public ChassisSpeeds toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            return speeds;
        }

        @Override
        public Rotation2d getTareRotation() {
            // Will no-op downstream.
            return null;
        }
    };

    /**
     * Converts perspective relative speeds to robot relative speeds.
     * @param speeds The perspective relative speeds to convert.
     * @param robotAngle The blue origin relative angle of the robot.
     */
    public abstract ChassisSpeeds toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

    /**
     * Converts robot relative speeds to the perspective relative speeds.
     * @param speeds The robot relative speeds to convert.
     * @param robotAngle The blue origin relative angle of the robot.
     */
    public abstract ChassisSpeeds toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

    /**
     * Gets the rotation to apply as the new zero when
     * taring the robot's rotation to the perspective.
     */
    public abstract Rotation2d getTareRotation();
}
