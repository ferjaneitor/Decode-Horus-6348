package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionStdDevModel {

    private final double linearStdDevBaselineMeters;
    private final double angularStdDevBaselineRadians;
    private final double maximumAmbiguityForSingleTag;
    private final double maximumZErrorMeters;

    public VisionStdDevModel(
            double linearStdDevBaselineMeters,
            double angularStdDevBaselineRadians,
            double maximumAmbiguityForSingleTag,
            double maximumZErrorMeters
    ) {
        this.linearStdDevBaselineMeters = linearStdDevBaselineMeters;
        this.angularStdDevBaselineRadians = angularStdDevBaselineRadians;
        this.maximumAmbiguityForSingleTag = maximumAmbiguityForSingleTag;
        this.maximumZErrorMeters = maximumZErrorMeters;
    }

    public boolean shouldReject(VisionObservation observation, double fieldLengthMeters, double fieldWidthMeters) {
        if (observation.tagCount() <= 0) {
            return true;
        }

        if (observation.tagCount() == 1 && observation.ambiguity() > maximumAmbiguityForSingleTag) {
            return true;
        }

        if (Math.abs(observation.robotPose().getZ()) > maximumZErrorMeters) {
            return true;
        }

        double x = observation.robotPose().getX();
        double y = observation.robotPose().getY();
        return x < 0.0 || x > fieldLengthMeters || y < 0.0 || y > fieldWidthMeters;
    }

    public Matrix<N3, N1> calculateStdDevs(VisionObservation observation, double cameraFactor, PoseEstimateSourceEnum level) {
        double distanceMeters = Math.max(0.001, observation.averageTagDistanceMeters());
        int tagCount = Math.max(1, observation.tagCount());

        double stdDevFactor = (distanceMeters * distanceMeters) / tagCount;

        double linearStdDev = linearStdDevBaselineMeters * stdDevFactor * cameraFactor;
        double angularStdDev = angularStdDevBaselineRadians * stdDevFactor * cameraFactor;

        if (!observation.visionRotationTrusted()) {
            angularStdDev = Double.POSITIVE_INFINITY;
        }

        // Use enum as multiplier, not as replacement (keeps "two worlds" together)
        // LOW -> smaller multiplier; HIGH -> bigger multiplier; NONE -> basically ignore
        double levelMultiplier = switch (level) {
            case LOW -> 0.5;
            case MEDIUM -> 1.0;
            case HIGH -> 2.0;
            case NONE -> 1e6;
        };

        return VecBuilder.fill(
                linearStdDev * levelMultiplier,
                linearStdDev * levelMultiplier,
                angularStdDev * levelMultiplier
        );
    }
}
