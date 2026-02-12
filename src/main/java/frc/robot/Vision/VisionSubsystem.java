package frc.robot.Vision;


import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.SuperSubsystem.SuperVision.PoseEstimateSourceEnum;
import frc.SuperSubsystem.SuperVision.VisionIO;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final VisionConsumer visionConsumer;

    private final VisionIO[] visionInputOutputArray;
    private final VisionIOInputsAutoLogged[] visionInputsAutoLoggedArray;

    private final PoseEstimateSourceEnum[] cameraTrustLevels;
    private final Alert[] disconnectedCameraAlerts;

    public VisionSubsystem(
            AprilTagFieldLayout aprilTagFieldLayout,
            VisionConsumer visionConsumer,
            PoseEstimateSourceEnum[] cameraTrustLevels,
            VisionIO... visionInputOutputArray
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.visionConsumer = visionConsumer;
        this.cameraTrustLevels = cameraTrustLevels;

        this.visionInputOutputArray = visionInputOutputArray;
        this.visionInputsAutoLoggedArray = new VisionIOInputsAutoLogged[visionInputOutputArray.length];
        this.disconnectedCameraAlerts = new Alert[visionInputOutputArray.length];

        for (int cameraIndex = 0; cameraIndex < visionInputOutputArray.length; cameraIndex++) {
            visionInputsAutoLoggedArray[cameraIndex] = new VisionIOInputsAutoLogged();
            disconnectedCameraAlerts[cameraIndex] = new Alert(
                    "Vision camera " + cameraIndex + " is disconnected.",
                    AlertType.kWarning
            );
        }
    }

    @Override
    public void periodic() {
        List<Pose3d> acceptedRobotPoses = new ArrayList<>();
        List<Pose3d> rejectedRobotPoses = new ArrayList<>();

        for (int cameraIndex = 0; cameraIndex < visionInputOutputArray.length; cameraIndex++) {
            VisionIO visionInputOutput = visionInputOutputArray[cameraIndex];
            VisionIOInputsAutoLogged inputs = visionInputsAutoLoggedArray[cameraIndex];

            visionInputOutput.updateInputs(inputs);
            Logger.processInputs("Vision/Camera" + cameraIndex, inputs);

            disconnectedCameraAlerts[cameraIndex].set(!inputs.cameraConnected);

            PoseEstimateSourceEnum trustLevel =
                    (cameraIndex < cameraTrustLevels.length) ? cameraTrustLevels[cameraIndex] : PoseEstimateSourceEnum.MEDIUM;

            double cameraFactor =
                    (cameraIndex < VisionConstants.cameraStandardDeviationFactors.length) ? VisionConstants.cameraStandardDeviationFactors[cameraIndex] : 1.0;

            for (VisionIO.PoseObservation poseObservation : inputs.poseObservations) {
                boolean rejected = shouldRejectPose(poseObservation);
                if (rejected) {
                    rejectedRobotPoses.add(poseObservation.robotPose());
                    continue;
                }

                acceptedRobotPoses.add(poseObservation.robotPose());

                Matrix<N3, N1> standardDeviations = calculateStandardDeviations(poseObservation, cameraFactor, trustLevel);

                visionConsumer.accept(
                        poseObservation.robotPose().toPose2d(),
                        poseObservation.timestampSeconds(),
                        standardDeviations
                );
            }
        }

        Logger.recordOutput("Vision/Summary/AcceptedRobotPoses", acceptedRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Summary/RejectedRobotPoses", rejectedRobotPoses.toArray(Pose3d[]::new));
    }

    private boolean shouldRejectPose(VisionIO.PoseObservation poseObservation) {
        if (poseObservation.tagCount() <= 0) {
            return true;
        }

        if (poseObservation.tagCount() == 1 && poseObservation.ambiguity() > VisionConstants.maximumAmbiguityForSingleTag) {
            return true;
        }

        if (Math.abs(poseObservation.robotPose().getZ()) > VisionConstants.maximumAbsoluteZErrorMeters) {
            return true;
        }

        double robotX = poseObservation.robotPose().getX();
        double robotY = poseObservation.robotPose().getY();

        return robotX < 0.0
                || robotX > aprilTagFieldLayout.getFieldLength()
                || robotY < 0.0
                || robotY > aprilTagFieldLayout.getFieldWidth();
    }

    private Matrix<N3, N1> calculateStandardDeviations(
            VisionIO.PoseObservation poseObservation,
            double cameraFactor,
            PoseEstimateSourceEnum trustLevel
    ) {
        double distanceMeters = Math.max(0.001, poseObservation.averageTagDistanceMeters());
        int tagCount = Math.max(1, poseObservation.tagCount());

        double baseFactor = (distanceMeters * distanceMeters) / tagCount;

        double linearStdDevMeters = VisionConstants.linearStdDevBaselineMeters * baseFactor * cameraFactor;
        double angularStdDevRadians = VisionConstants.angularStdDevBaselineRadians * baseFactor * cameraFactor;

        if (!poseObservation.visionRotationTrusted()) {
            angularStdDevRadians = Double.POSITIVE_INFINITY;
        }

        double trustMultiplier = switch (trustLevel) {
            case LOW -> 0.5;
            case MEDIUM -> 1.0;
            case HIGH -> 2.0;
            case NONE -> 1e6;
        };

        return VecBuilder.fill(
                linearStdDevMeters * trustMultiplier,
                linearStdDevMeters * trustMultiplier,
                angularStdDevRadians * trustMultiplier
        );
    }
}
