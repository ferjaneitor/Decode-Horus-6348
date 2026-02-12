package frc.AutoLogger;

import java.util.Arrays;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import frc.SuperSubsystem.SuperVision.VisionIO;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("CameraConnected", cameraConnected);

        table.put("LatestTargetYawRadians", latestTargetYawRadians);
        table.put("LatestTargetPitchRadians", latestTargetPitchRadians);

        table.put("ObservationTimestampsSeconds", observationTimestampsSeconds);
        table.put("ObservationRobotPoses", observationRobotPoses);
        table.put("ObservationAmbiguities", observationAmbiguities);
        table.put("ObservationTagCounts", observationTagCounts);
        table.put("ObservationAverageTagDistanceMeters", observationAverageTagDistanceMeters);
        table.put("ObservationRotationTrusted", observationRotationTrusted);
        table.put("ObservationTypeOrdinals", observationTypeOrdinals);

        table.put("DetectedTagIdentifiers", detectedTagIdentifiers);

        table.put("FramesPerSecond", framesPerSecond);
    }

    @Override
    public void fromLog(LogTable table) {
        cameraConnected = table.get("CameraConnected", cameraConnected);

        latestTargetYawRadians = table.get("LatestTargetYawRadians", latestTargetYawRadians);
        latestTargetPitchRadians = table.get("LatestTargetPitchRadians", latestTargetPitchRadians);

        observationTimestampsSeconds = table.get("ObservationTimestampsSeconds", observationTimestampsSeconds);
        observationRobotPoses = table.get("ObservationRobotPoses", observationRobotPoses);
        observationAmbiguities = table.get("ObservationAmbiguities", observationAmbiguities);
        observationTagCounts = table.get("ObservationTagCounts", observationTagCounts);
        observationAverageTagDistanceMeters =
                table.get("ObservationAverageTagDistanceMeters", observationAverageTagDistanceMeters);
        observationRotationTrusted = table.get("ObservationRotationTrusted", observationRotationTrusted);
        observationTypeOrdinals = table.get("ObservationTypeOrdinals", observationTypeOrdinals);

        detectedTagIdentifiers = table.get("DetectedTagIdentifiers", detectedTagIdentifiers);

        framesPerSecond = table.get("FramesPerSecond", framesPerSecond);
    }

    @Override
    public VisionIOInputsAutoLogged clone() {
        VisionIOInputsAutoLogged copy;
        try {
            copy = (VisionIOInputsAutoLogged) super.clone();
        } catch (CloneNotSupportedException exception) {
            throw new AssertionError("Clone not supported", exception);
        }

        copy.cameraConnected = this.cameraConnected;

        copy.latestTargetYawRadians = this.latestTargetYawRadians;
        copy.latestTargetPitchRadians = this.latestTargetPitchRadians;

        copy.observationTimestampsSeconds =
                Arrays.copyOf(this.observationTimestampsSeconds, this.observationTimestampsSeconds.length);

        copy.observationRobotPoses =
                Arrays.copyOf(this.observationRobotPoses, this.observationRobotPoses.length);

        copy.observationAmbiguities =
                Arrays.copyOf(this.observationAmbiguities, this.observationAmbiguities.length);

        copy.observationTagCounts =
                Arrays.copyOf(this.observationTagCounts, this.observationTagCounts.length);

        copy.observationAverageTagDistanceMeters =
                Arrays.copyOf(this.observationAverageTagDistanceMeters, this.observationAverageTagDistanceMeters.length);

        copy.observationRotationTrusted =
                Arrays.copyOf(this.observationRotationTrusted, this.observationRotationTrusted.length);

        copy.observationTypeOrdinals =
                Arrays.copyOf(this.observationTypeOrdinals, this.observationTypeOrdinals.length);

        copy.detectedTagIdentifiers =
                Arrays.copyOf(this.detectedTagIdentifiers, this.detectedTagIdentifiers.length);

        copy.framesPerSecond = this.framesPerSecond;

        return copy;
    }
}
