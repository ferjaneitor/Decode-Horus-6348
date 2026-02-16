package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.AutoLogger.VisionIOInputsAutoLogged;
import frc.SuperSubsystem.SuperVision.LoggedVisionCamera;
import frc.SuperSubsystem.SuperVision.VisionEntries;
import frc.SuperSubsystem.SuperVision.VisionEnums;
import frc.SuperSubsystem.SuperVision.VisionIO;
import frc.SuperSubsystem.SuperVision.VisionStandardDeviationModel;
import frc.robot.Constants.FieldCosntants;

public class VisionSubsystem extends SubsystemBase {

    public interface VisionPoseMeasurementConsumer {
        void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStandardDeviations);
    }

    public interface VisionHardwareFactory {
        VisionIO createVisionHardware(VisionEntries.CameraSpecifications cameraSpecifications, AprilTagFieldLayout aprilTagFieldLayout);
    }

    private final List<LoggedVisionCamera> loggedVisionCameraList = new ArrayList<>();

    private final Supplier<Pose2d> currentRobotPoseSupplier;
    private final Supplier<Double> yawRateRadiansPerSecondSupplier;
    private final VisionPoseMeasurementConsumer visionPoseMeasurementConsumer;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final VisionStandardDeviationModel visionStandardDeviationModel;

    private final double fieldLengthMeters;
    private final double fieldWidthMeters;

    private boolean visionEnabled = true;

    private boolean isShootingTargetValidThisCycle = false;

    public VisionSubsystem(
            AprilTagFieldLayout aprilTagFieldLayout,
            double fieldLengthMeters,
            double fieldWidthMeters,
            Supplier<Pose2d> currentRobotPoseSupplier,
            Supplier<Double> yawRateRadiansPerSecondSupplier,
            VisionPoseMeasurementConsumer visionPoseMeasurementConsumer,
            VisionStandardDeviationModel visionStandardDeviationModel,
            List<VisionEntries.CameraSpecifications> cameraSpecificationsList,
            VisionHardwareFactory visionHardwareFactory
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.fieldLengthMeters = fieldLengthMeters;
        this.fieldWidthMeters = fieldWidthMeters;
        this.currentRobotPoseSupplier = currentRobotPoseSupplier;
        this.yawRateRadiansPerSecondSupplier = yawRateRadiansPerSecondSupplier;
        this.visionPoseMeasurementConsumer = visionPoseMeasurementConsumer;
        this.visionStandardDeviationModel = visionStandardDeviationModel;

        for (VisionEntries.CameraSpecifications cameraSpecifications : cameraSpecificationsList) {
            VisionIO visionHardwareInterface = visionHardwareFactory.createVisionHardware(cameraSpecifications, aprilTagFieldLayout);

            loggedVisionCameraList.add(
                    new LoggedVisionCamera(
                            cameraSpecifications.cameraName(),
                            visionHardwareInterface,
                            cameraSpecifications.baseNoiseLevel(),
                            cameraSpecifications.cameraConfidenceMultiplier()
                    )
            );
        }
    }

    @Override
    public void periodic() {
        if (!visionEnabled || loggedVisionCameraList.isEmpty()) {
            return;
        }

        Pose2d currentRobotPose2d = currentRobotPoseSupplier.get();
        Pose3d currentRobotPose3d = new Pose3d(currentRobotPose2d);

        double currentRobotTimestampSeconds = Timer.getFPGATimestamp();
        double yawRateRadiansPerSecond = yawRateRadiansPerSecondSupplier.get();

        boolean itsAValidShootingTarget = false;

        long[] validShootingTagIds = FieldCosntants.getShootingValidTagIdentifiers();

        for (LoggedVisionCamera loggedVisionCamera : loggedVisionCameraList) {
            loggedVisionCamera.update(currentRobotPose3d);

            VisionIOInputsAutoLogged visionInputs = loggedVisionCamera.getVisionInputs();

            List<Pose3d> visibleTagPoseList = new ArrayList<>();
            for (long detectedTagIdentifier : visionInputs.detectedTagIdentifiers) {
                Optional<Pose3d> fieldToTagPoseOptional = aprilTagFieldLayout.getTagPose((int) detectedTagIdentifier);
                fieldToTagPoseOptional.ifPresent(visibleTagPoseList::add);
                for (long validShootingTagId : validShootingTagIds) {
                    if (detectedTagIdentifier == validShootingTagId) {
                        itsAValidShootingTarget = true;
                        break;
                    }
                }
                if (itsAValidShootingTarget) {
                    break;
                }
            }

            this.isShootingTargetValidThisCycle = itsAValidShootingTarget;

            int observationCount = visionInputs.observationTimestampsSeconds.length;

            Pose3d[] rawRobotPoseEstimateArray = new Pose3d[observationCount];
            List<Pose3d> acceptedRobotPoseEstimateList = new ArrayList<>();
            List<Pose3d> rejectedRobotPoseEstimateList = new ArrayList<>();

            VisionEnums.VisionRejectReason lastRejectReason = null;

            for (int observationIndex = 0; observationIndex < observationCount; observationIndex++) {
                Pose3d robotPose = visionInputs.observationRobotPoses[observationIndex];
                rawRobotPoseEstimateArray[observationIndex] = robotPose;

                int tagCount = (int) visionInputs.observationTagCounts[observationIndex];

                int observationTypeOrdinal = (int) visionInputs.observationTypeOrdinals[observationIndex];
                VisionEnums.PoseObservationType observationType =
                        VisionEnums.PoseObservationType.values()[
                                Math.max(0, Math.min(observationTypeOrdinal, VisionEnums.PoseObservationType.values().length - 1))
                        ];

                VisionEntries.VisionObservation visionObservation = new VisionEntries.VisionObservation(
                        loggedVisionCamera.getCameraName(),
                        visionInputs.observationTimestampsSeconds[observationIndex],
                        robotPose,
                        visionInputs.observationAmbiguities[observationIndex],
                        tagCount,
                        visionInputs.observationAverageTagDistanceMeters[observationIndex],
                        visionInputs.observationRotationTrusted[observationIndex],
                        observationType
                );

                if (!visionInputs.photonPoseEstimatorEnabled) {
                    rejectedRobotPoseEstimateList.add(robotPose);
                    lastRejectReason = VisionEnums.VisionRejectReason.PHOTON_POSE_ESTIMATOR_DISABLED;
                    continue;
                }

                VisionEnums.VisionRejectReason rejectReason = visionStandardDeviationModel.getRejectReasonOrNull(
                        visionObservation,
                        currentRobotTimestampSeconds,
                        yawRateRadiansPerSecond,
                        fieldLengthMeters,
                        fieldWidthMeters
                );

                if (rejectReason != null) {
                    rejectedRobotPoseEstimateList.add(robotPose);
                    lastRejectReason = rejectReason;
                    continue;
                }

                Matrix<N3, N1> baseStandardDeviationMatrix =
                        loggedVisionCamera.getBaseNoiseLevel().getBaseStandardDeviationMatrix();

                Matrix<N3, N1> visionMeasurementStandardDeviations =
                        visionStandardDeviationModel.calculateStandardDeviations(
                                visionObservation,
                                baseStandardDeviationMatrix,
                                loggedVisionCamera.getCameraConfidenceMultiplier()
                        );

                visionPoseMeasurementConsumer.addVisionMeasurement(
                        robotPose.toPose2d(),
                        visionObservation.timestampSeconds(),
                        visionMeasurementStandardDeviations
                );

                acceptedRobotPoseEstimateList.add(robotPose);
            }

            Pose3d[] visibleTagPoseArray = visibleTagPoseList.toArray(Pose3d[]::new);
            Pose3d[] acceptedRobotPoseEstimateArray = acceptedRobotPoseEstimateList.toArray(Pose3d[]::new);
            Pose3d[] rejectedRobotPoseEstimateArray = rejectedRobotPoseEstimateList.toArray(Pose3d[]::new);

            loggedVisionCamera.recordCameraOutputs(
                    visibleTagPoseArray,
                    rawRobotPoseEstimateArray,
                    acceptedRobotPoseEstimateArray,
                    rejectedRobotPoseEstimateArray,
                    lastRejectReason
            );
        }
    }

    public double getLatestTargetYawRadians() {
        // Si tienes varias cámaras, aquí decides cuál usar (por ejemplo la primera, o la que tenga target).
        // Por ahora asumimos la primera.
        if (loggedVisionCameraList.isEmpty()) {
            return 0.0;
        }
        return loggedVisionCameraList.get(0).getVisionInputs().latestTargetYawRadians;
    }

    public boolean hasTarget() {
        for (LoggedVisionCamera loggedVisionCamera : loggedVisionCameraList) {
            if (loggedVisionCamera.getVisionInputs().hasTarget) {
                return true;
            }
        }
        return false;
    }

    public void setVisionEnabled(boolean visionEnabled) {
        this.visionEnabled = visionEnabled;
    }

    public boolean itsAValidShootingTarget() {
        return this.isShootingTargetValidThisCycle;
    }
}
