package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {

    protected final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private VisionEnums.PoseEstimationMode poseEstimationMode = VisionEnums.PoseEstimationMode.COPROCESSOR_MULTI_TAG;
    private Pose3d referencePoseForEstimation = new Pose3d();

    public VisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCameraTransform3d,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {
        this.photonCamera = new PhotonCamera(cameraName);

        // 2026: preferred constructor is (fieldTags, robotToCamera). :contentReference[oaicite:3]{index=3}
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCameraTransform3d);
    }

    public void setPoseEstimationMode(VisionEnums.PoseEstimationMode poseEstimationMode) {
        this.poseEstimationMode = poseEstimationMode;
    }

    @Override
    public void setReferencePoseForEstimation(Pose3d referencePose) {
        this.referencePoseForEstimation = referencePose;
    }

    @Override
    public void setDriverMode(boolean driverModeEnabled) {
        photonCamera.setDriverMode(driverModeEnabled);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.cameraConnected = photonCamera.isConnected();
        inputs.photonPoseEstimatorEnabled = true;

        List<PhotonPipelineResult> unreadPipelineResults = photonCamera.getAllUnreadResults();
        if (unreadPipelineResults == null || unreadPipelineResults.isEmpty()) {
            inputs.observationTimestampsSeconds = new double[0];
            inputs.observationRobotPoses = new Pose3d[0];
            inputs.observationAmbiguities = new double[0];
            inputs.observationTagCounts = new long[0];
            inputs.observationAverageTagDistanceMeters = new double[0];
            inputs.observationRotationTrusted = new boolean[0];
            inputs.observationTypeOrdinals = new long[0];
            inputs.detectedTagIdentifiers = new long[0];
            return;
        }

        Set<Integer> detectedTagIdentifierSet = new HashSet<>();

        List<Double> timestampList = new ArrayList<>();
        List<Pose3d> robotPoseList = new ArrayList<>();
        List<Double> ambiguityList = new ArrayList<>();
        List<Long> tagCountList = new ArrayList<>();
        List<Double> averageDistanceList = new ArrayList<>();
        List<Boolean> rotationTrustedList = new ArrayList<>();
        List<Long> observationTypeOrdinalList = new ArrayList<>();

        for (PhotonPipelineResult pipelineResult : unreadPipelineResults) {
            if (pipelineResult == null) {
                continue;
            }

            if (pipelineResult.hasTargets()) {
                var bestTarget = pipelineResult.getBestTarget();
                inputs.latestTargetYawRadians = Math.toRadians(bestTarget.getYaw());
                inputs.latestTargetPitchRadians = Math.toRadians(bestTarget.getPitch());
            } else {
                inputs.latestTargetYawRadians = 0.0;
                inputs.latestTargetPitchRadians = 0.0;
            }

            Optional<EstimatedRobotPose> estimatedRobotPoseOptional = estimateRobotPoseWithFallback(pipelineResult);
            if (estimatedRobotPoseOptional.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();
            Pose3d estimatedRobotPose3d = estimatedRobotPose.estimatedPose;

            int tagCount = (estimatedRobotPose.targetsUsed != null) ? estimatedRobotPose.targetsUsed.size() : 0;
            if (tagCount <= 0) {
                continue;
            }

            double totalDistanceMeters = 0.0;
            double poseAmbiguity = 0.0;

            for (var target : estimatedRobotPose.targetsUsed) {
                detectedTagIdentifierSet.add(target.getFiducialId());
                totalDistanceMeters += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            totalDistanceMeters /= tagCount;

            if (tagCount == 1) {
                poseAmbiguity = estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity();
            }

            boolean rotationTrusted = tagCount > 1;

            VisionEnums.PoseObservationType observationType =
                    (tagCount > 1)
                            ? VisionEnums.PoseObservationType.PHOTONVISION_MULTI_TAG
                            : VisionEnums.PoseObservationType.PHOTONVISION_SINGLE_TAG;

            timestampList.add(estimatedRobotPose.timestampSeconds);
            robotPoseList.add(estimatedRobotPose3d);
            ambiguityList.add(poseAmbiguity);
            tagCountList.add((long) tagCount);
            averageDistanceList.add(totalDistanceMeters);
            rotationTrustedList.add(rotationTrusted);
            observationTypeOrdinalList.add((long) observationType.ordinal());
        }

        inputs.observationTimestampsSeconds = timestampList.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.observationRobotPoses = robotPoseList.toArray(Pose3d[]::new);
        inputs.observationAmbiguities = ambiguityList.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.observationTagCounts = tagCountList.stream().mapToLong(Long::longValue).toArray();
        inputs.observationAverageTagDistanceMeters = averageDistanceList.stream().mapToDouble(Double::doubleValue).toArray();

        inputs.observationRotationTrusted = new boolean[rotationTrustedList.size()];
        for (int observationIndex = 0; observationIndex < rotationTrustedList.size(); observationIndex++) {
            inputs.observationRotationTrusted[observationIndex] = rotationTrustedList.get(observationIndex);
        }

        inputs.observationTypeOrdinals = observationTypeOrdinalList.stream().mapToLong(Long::longValue).toArray();

        long[] detectedTagIdentifierArray = new long[detectedTagIdentifierSet.size()];
        int detectedTagIndex = 0;
        for (int detectedTagIdentifier : detectedTagIdentifierSet) {
            detectedTagIdentifierArray[detectedTagIndex++] = detectedTagIdentifier;
        }
        inputs.detectedTagIdentifiers = detectedTagIdentifierArray;
    }

    private Optional<EstimatedRobotPose> estimateRobotPoseWithFallback(PhotonPipelineResult pipelineResult) {
        Optional<EstimatedRobotPose> coprocessorMultiTagEstimate =
                photonPoseEstimator.estimateCoprocMultiTagPose(pipelineResult);

        if (coprocessorMultiTagEstimate.isPresent()) {
            return coprocessorMultiTagEstimate;
        }

        return switch (poseEstimationMode) {
            case COPROCESSOR_MULTI_TAG -> photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);
            case LOWEST_AMBIGUITY -> photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);
            case CLOSEST_TO_REFERENCE_POSE ->
                    photonPoseEstimator.estimateClosestToReferencePose(pipelineResult, referencePoseForEstimation);
            case AVERAGE_BEST_TARGETS -> photonPoseEstimator.estimateAverageBestTargetsPose(pipelineResult);
        };
    }
}
