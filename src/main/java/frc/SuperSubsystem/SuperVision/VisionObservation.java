package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionObservation(
        double timestampSeconds,
        Pose3d robotPose,
        double ambiguity,
        int tagCount,
        double averageTagDistanceMeters,
        boolean visionRotationTrusted
) {}
