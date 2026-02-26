package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.SuperSubsystem.SuperVision.VisionEntries;
import frc.SuperSubsystem.SuperVision.VisionIO;
import frc.SuperSubsystem.SuperVision.VisionIOPhotonVision;
import frc.SuperSubsystem.SuperVision.VisionIOPhotonVisionSim;

public final class VisionHardwareFactoryImpl implements VisionSubsystem.VisionHardwareFactory {

    private final boolean isSimulation;

    public VisionHardwareFactoryImpl(boolean isSimulation) {
        this.isSimulation = isSimulation;
    }

    @Override
    public VisionIO createVisionHardware(
            VisionEntries.CameraSpecifications cameraSpecifications,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {
        if (isSimulation) {
            return new VisionIOPhotonVisionSim(
                    cameraSpecifications.cameraName(),
                    cameraSpecifications.getRobotToCameraTransform3d(),
                    aprilTagFieldLayout
            );
        }

        return new VisionIOPhotonVision(
                cameraSpecifications.cameraName(),
                cameraSpecifications.getRobotToCameraTransform3d(),
                aprilTagFieldLayout
        );
    }
}