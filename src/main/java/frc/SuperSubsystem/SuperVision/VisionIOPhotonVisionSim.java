package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

    private static VisionSystemSim visionSystemSimulation;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final PhotonCameraSim photonCameraSimulation;

    public VisionIOPhotonVisionSim(
            String cameraName,
            Transform3d robotToCameraTransform,
            AprilTagFieldLayout aprilTagFieldLayout,
            Supplier<Pose2d> robotPoseSupplier
    ) {
        super(cameraName, robotToCameraTransform, aprilTagFieldLayout);
        this.robotPoseSupplier = robotPoseSupplier;

        if (visionSystemSimulation == null) {
            visionSystemSimulation = new VisionSystemSim("main");
            visionSystemSimulation.addAprilTags(aprilTagFieldLayout);
        }

        SimCameraProperties simulationCameraProperties = new SimCameraProperties();
        photonCameraSimulation = new PhotonCameraSim(getPhotonCameraForSimulation(cameraName), simulationCameraProperties, aprilTagFieldLayout);

        visionSystemSimulation.addCamera(photonCameraSimulation, robotToCameraTransform);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSystemSimulation.update(robotPoseSupplier.get());
        super.updateInputs(inputs);
    }

    private static org.photonvision.PhotonCamera getPhotonCameraForSimulation(String cameraName) {
        return new org.photonvision.PhotonCamera(cameraName);
    }
}
