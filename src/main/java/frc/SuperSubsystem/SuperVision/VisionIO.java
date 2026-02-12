package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public boolean cameraConnected = false;

        public TargetObservation latestTargetObservation =
                new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] detectedTagIdentifiers = new int[0];

        public long framesPerSecond = 0;
    }

    record TargetObservation(Rotation2d yawToTarget, Rotation2d pitchToTarget) {}

    record PoseObservation(
            double timestampSeconds,
            Pose3d robotPose,
            double ambiguity,
            int tagCount,
            double averageTagDistanceMeters,
            PoseObservationType observationType,
            boolean visionRotationTrusted
    ) {}

    enum PoseObservationType {
        PHOTONVISION_MULTI_TAG,
        PHOTONVISION_SINGLE_TAG
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void setDriverMode(boolean driverModeEnabled) {}
}
