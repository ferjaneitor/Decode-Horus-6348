package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Objects;

public final class VisionEntries {
    private VisionEntries() {}

    public record CameraSpecifications(
            String cameraName,
            Transform3d cameraToRobotTransform3d,
            VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel,
            double cameraConfidenceMultiplier
    ) {
        public CameraSpecifications {
            Objects.requireNonNull(cameraName);
            Objects.requireNonNull(cameraToRobotTransform3d);
            Objects.requireNonNull(baseNoiseLevel);
        }

        public Transform3d getRobotToCameraTransform3d() {
            return cameraToRobotTransform3d.inverse();
        }
    }

    /** Normalized observation (minimum useful data for filtering + std devs + logging). */
    public record VisionObservation(
            String cameraName,
            double timestampSeconds,
            Pose3d robotPose,
            double ambiguity,
            int tagCount,
            double averageTagDistanceMeters,
            boolean visionRotationTrusted,
            VisionEnums.PoseObservationType observationType
    ) {}

    public record VisionFilterResult(
            boolean accepted,
            VisionEnums.VisionRejectReason rejectReasonOrNull,
            Matrix<N3, N1> visionMeasurementStandardDeviationsOrNull
    ) {}
}
