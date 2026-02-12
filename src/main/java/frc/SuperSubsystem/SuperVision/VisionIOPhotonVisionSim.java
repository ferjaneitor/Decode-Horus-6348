package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

    private static VisionSystemSim visionSystemSimulation;

    private final Supplier<Pose2d> robotPoseSupplier;
    //@SuppressWarnings("unused")
    private final PhotonCameraSim photonCameraSimulation;

    public VisionIOPhotonVisionSim(
            String cameraName,
            Transform3d robotToCameraTransform3d,
            AprilTagFieldLayout aprilTagFieldLayout,
            Supplier<Pose2d> robotPoseSupplier
    ) {
        super(cameraName, robotToCameraTransform3d, aprilTagFieldLayout);

        this.robotPoseSupplier = robotPoseSupplier;

        if (visionSystemSimulation == null) {
            visionSystemSimulation = new VisionSystemSim("main");
            visionSystemSimulation.addAprilTags(aprilTagFieldLayout);
        }

        SimCameraProperties simulationCameraProperties = new SimCameraProperties();
        PhotonCamera photonCameraHandle = new PhotonCamera(cameraName);

        photonCameraSimulation = new PhotonCameraSim(photonCameraHandle, simulationCameraProperties, aprilTagFieldLayout);
        visionSystemSimulation.addCamera(photonCameraSimulation, robotToCameraTransform3d);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSystemSimulation.update(robotPoseSupplier.get());
        super.updateInputs(inputs);
    }
}
