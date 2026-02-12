package frc.SuperSubsystem.SuperVision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO {

    private final PhotonCamera photonCamera;
    private final Transform3d robotToCameraTransform;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private PhotonPoseEstimator photonPoseEstimator;
    private PoseEstimationModeEnum poseEstimationMode;
    private Pose3d referencePoseForEstimation;

    public VisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCameraTransform,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.robotToCameraTransform = robotToCameraTransform;
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        this.photonPoseEstimator = null;
        this.poseEstimationMode = PoseEstimationModeEnum.COPROCESSOR_MULTI_TAG;
        this.referencePoseForEstimation = new Pose3d();
    }

    public void enablePoseEstimation() {
        // NOTE: constructor/methods depend on PhotonVision version.
        // If this line does not compile in your project, we adjust it to your PV version.
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCameraTransform);
    }

    public void setPoseEstimationMode(PoseEstimationModeEnum mode) {
        this.poseEstimationMode = mode;
    }

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

        Set<Integer> detectedTagIdentifiers = new HashSet<>();
        List<PoseObservation> poseObservationList = new ArrayList<>();

        for (PhotonPipelineResult pipelineResult : photonCamera.getAllUnreadResults()) {
            // Latest best target observation (aiming)
            if (pipelineResult.hasTargets()) {
                var bestTarget = pipelineResult.getBestTarget();
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(bestTarget.getYaw()),
                        Rotation2d.fromDegrees(bestTarget.getPitch())
                );
            } else {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
            }

            // Pose observation (preferred path = PhotonPoseEstimator)
            Optional<PoseObservation> poseObservationOptional = buildPoseObservation(pipelineResult, detectedTagIdentifiers);
            poseObservationOptional.ifPresent(poseObservationList::add);
        }

        inputs.poseObservations = poseObservationList.toArray(PoseObservation[]::new);

        int[] tagIdentifierArray = new int[detectedTagIdentifiers.size()];
        int tagIndex = 0;
        for (int detectedTagIdentifier : detectedTagIdentifiers) {
            tagIdentifierArray[tagIndex++] = detectedTagIdentifier;
        }
        inputs.detectedTagIdentifiers = tagIdentifierArray;
    }

    private Optional<PoseObservation> buildPoseObservation(
            PhotonPipelineResult pipelineResult,
            Set<Integer> detectedTagIdentifiers
    ) {
        if (photonPoseEstimator == null) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedRobotPoseOptional = estimateRobotPose(pipelineResult);
        if (estimatedRobotPoseOptional.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();

        Pose3d robotPose = estimatedRobotPose.estimatedPose;
        double timestampSeconds = estimatedRobotPose.timestampSeconds;

        int tagCount = (estimatedRobotPose.targetsUsed != null) ? estimatedRobotPose.targetsUsed.size() : 0;

        double totalDistanceMeters = 0.0;
        double ambiguity = 0.0;

        if (estimatedRobotPose.targetsUsed != null && !estimatedRobotPose.targetsUsed.isEmpty()) {
            for (var target : estimatedRobotPose.targetsUsed) {
                detectedTagIdentifiers.add(target.getFiducialId());
                totalDistanceMeters += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            totalDistanceMeters /= estimatedRobotPose.targetsUsed.size();

            if (estimatedRobotPose.targetsUsed.size() == 1) {
                ambiguity = estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity();
            }
        }

        boolean visionRotationTrusted = tagCount > 1;

        VisionIO.PoseObservationType observationType =
                (tagCount > 1)
                        ? VisionIO.PoseObservationType.PHOTONVISION_MULTI_TAG
                        : VisionIO.PoseObservationType.PHOTONVISION_SINGLE_TAG;

        return Optional.of(
                new PoseObservation(
                        timestampSeconds,
                        robotPose,
                        ambiguity,
                        tagCount,
                        totalDistanceMeters,
                        observationType,
                        visionRotationTrusted
                )
        );
    }

    private Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult pipelineResult) {
        return switch (poseEstimationMode) {
            case COPROCESSOR_MULTI_TAG -> photonPoseEstimator.estimateCoprocMultiTagPose(pipelineResult);
            case LOWEST_AMBIGUITY -> photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);
            case CLOSEST_TO_REFERENCE_POSE -> photonPoseEstimator.estimateClosestToReferencePose(
                    pipelineResult, referencePoseForEstimation
            );
            case AVERAGE_BEST_TARGETS -> photonPoseEstimator.estimateAverageBestTargetsPose(pipelineResult);
        };
    }
}
