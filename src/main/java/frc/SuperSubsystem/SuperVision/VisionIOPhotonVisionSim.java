package frc.SuperSubsystem.SuperVision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

    private static VisionSystemSim visionSystemSimulation;

    private final Supplier<Pose2d> robotPoseSupplier;
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

        SimCameraProperties simulationCameraProperties = createReasonableSimulationCameraProperties();

        // IMPORTANT:
        // Use the same PhotonCamera instance that VisionIOPhotonVision owns.
        // This avoids NT name collisions and “two handles for one camera” weirdness.
        this.photonCameraSimulation =
                new PhotonCameraSim(this.photonCamera, simulationCameraProperties, aprilTagFieldLayout);

        visionSystemSimulation.addCamera(this.photonCameraSimulation, robotToCameraTransform3d);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d robotPose = robotPoseSupplier.get();

        // PhotonVision sim can occasionally hit a degenerate geometry case (Rotation2d x=y=0).
        // Do NOT allow that to crash the entire robot loop.
        try {
            visionSystemSimulation.update(robotPose);
        } catch (Exception exception) {
            // Leave inputs as-is; super.updateInputs will still read any valid frames
            // from previous cycles if they exist.
            // If you want, you can add a Logger line here.
        }

        super.updateInputs(inputs);
    }

    private static SimCameraProperties createReasonableSimulationCameraProperties() {
        SimCameraProperties simulationCameraProperties = new SimCameraProperties();

        // These values do not need to be perfect; they just need to be non-degenerate and consistent.
        int imageWidthPixels = 1280;
        int imageHeightPixels = 720;
        Rotation2d diagonalFieldOfViewRadians = Rotation2d.fromDegrees(100.0);

        // NOTE: Depending on your PhotonVision version, method names may differ slightly.
        // If your IDE says a method doesn't exist, tell me your PhotonVision version and I’ll adapt it.
        simulationCameraProperties.setCalibration(imageWidthPixels, imageHeightPixels, diagonalFieldOfViewRadians);
        simulationCameraProperties.setFPS(30.0);
        simulationCameraProperties.setAvgLatencyMs(35.0);
        simulationCameraProperties.setLatencyStdDevMs(5.0);

        return simulationCameraProperties;
    }
}