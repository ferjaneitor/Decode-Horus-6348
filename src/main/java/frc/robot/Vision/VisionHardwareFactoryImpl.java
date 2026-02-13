package frc.robot.Vision;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.SuperSubsystem.SuperVision.VisionEntries;
import frc.SuperSubsystem.SuperVision.VisionIO;
import frc.SuperSubsystem.SuperVision.VisionIOPhotonVision;
import frc.SuperSubsystem.SuperVision.VisionIOPhotonVisionSim;

public final class VisionHardwareFactoryImpl implements VisionSubsystem.VisionHardwareFactory {

  private final Supplier<Pose2d> currentRobotPoseSupplier;
  private final boolean isSimulation;

  public VisionHardwareFactoryImpl(Supplier<Pose2d> currentRobotPoseSupplier, boolean isSimulation) {
    this.currentRobotPoseSupplier = currentRobotPoseSupplier;
    this.isSimulation = isSimulation;
  }

  @Override
  public VisionIO createVisionHardware(
      VisionEntries.CameraSpecifications cameraSpecifications,
      AprilTagFieldLayout aprilTagFieldLayout) {

    if (isSimulation) {
      return new VisionIOPhotonVisionSim(
          cameraSpecifications.cameraName(),
          cameraSpecifications.getRobotToCameraTransform3d(),
          aprilTagFieldLayout,
          currentRobotPoseSupplier);
    }

    return new VisionIOPhotonVision(
        cameraSpecifications.cameraName(),
        cameraSpecifications.getRobotToCameraTransform3d(),
        aprilTagFieldLayout);
  }
}
