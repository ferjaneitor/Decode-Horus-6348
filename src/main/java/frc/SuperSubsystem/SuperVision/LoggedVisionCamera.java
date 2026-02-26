package frc.SuperSubsystem.SuperVision;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import frc.AutoLogger.VisionIOInputsAutoLogged;

public final class LoggedVisionCamera {

    private final String cameraName;
    private final VisionIO visionHardwareInterface;

    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    private final VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel;
    private final double cameraConfidenceMultiplier;

    public LoggedVisionCamera(
            String cameraName,
            VisionIO visionHardwareInterface,
            VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel,
            double cameraConfidenceMultiplier
    ) {
        this.cameraName = Objects.requireNonNull(cameraName);
        this.visionHardwareInterface = Objects.requireNonNull(visionHardwareInterface);
        this.baseNoiseLevel = Objects.requireNonNull(baseNoiseLevel);
        this.cameraConfidenceMultiplier = cameraConfidenceMultiplier;
    }

    public void update(Pose3d referencePoseForEstimation) {
        visionHardwareInterface.setReferencePoseForEstimation(referencePoseForEstimation);
        visionHardwareInterface.updateInputs(visionInputs);

        Logger.processInputs("Vision/" + cameraName, visionInputs);
    }

    public VisionIOInputsAutoLogged getVisionInputs() {
        return visionInputs;
    }

    public String getCameraName() {
        return cameraName;
    }

    public VisionEnums.PoseEstimateNoiseLevel getBaseNoiseLevel() {
        return baseNoiseLevel;
    }

    public double getCameraConfidenceMultiplier() {
        return cameraConfidenceMultiplier;
    }

    public void recordCameraOutputs(
            Pose3d[] visibleTagPoseArray,
            Pose3d[] rawRobotPoseEstimateArray,
            Pose3d[] acceptedRobotPoseEstimateArray,
            Pose3d[] rejectedRobotPoseEstimateArray,
            VisionEnums.VisionRejectReason lastRejectReasonOrNull
    ) {
        Logger.recordOutput("Vision/" + cameraName + "/TagPoses", visibleTagPoseArray);
        Logger.recordOutput("Vision/" + cameraName + "/RobotPosesRaw", rawRobotPoseEstimateArray);
        Logger.recordOutput("Vision/" + cameraName + "/RobotPosesAccepted", acceptedRobotPoseEstimateArray);
        Logger.recordOutput("Vision/" + cameraName + "/RobotPosesRejected", rejectedRobotPoseEstimateArray);
        Logger.recordOutput(
                "Vision/" + cameraName + "/LastRejectReason",
                lastRejectReasonOrNull == null ? (String) null : lastRejectReasonOrNull.toString()
        );
    }
}