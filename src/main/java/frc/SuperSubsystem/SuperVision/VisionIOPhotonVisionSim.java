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

        // Reuse the same PhotonCamera instance owned by VisionIOPhotonVision.
        this.photonCameraSimulation =
                new PhotonCameraSim(this.photonCamera, simulationCameraProperties, aprilTagFieldLayout);

        visionSystemSimulation.addCamera(this.photonCameraSimulation, robotToCameraTransform3d);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d robotPose = robotPoseSupplier.get();

        boolean robotPoseIsFinite =
                Double.isFinite(robotPose.getX())
                        && Double.isFinite(robotPose.getY())
                        && Double.isFinite(robotPose.getRotation().getRadians());

        if (robotPoseIsFinite) {
            try {
                visionSystemSimulation.update(robotPose);
            } catch (Throwable throwable) {
                // Intentionally swallow to prevent the robot loop from dying in sim.
                // If you want, log throwable.getMessage() through AdvantageKit.
            }
        }

        super.updateInputs(inputs);
    }

    private static SimCameraProperties createReasonableSimulationCameraProperties() {
        SimCameraProperties simulationCameraProperties = new SimCameraProperties();

        int imageWidthPixels = 1280;
        int imageHeightPixels = 720;
        Rotation2d diagonalFieldOfView = Rotation2d.fromDegrees(100.0);

        simulationCameraProperties.setCalibration(imageWidthPixels, imageHeightPixels, diagonalFieldOfView);
        simulationCameraProperties.setFPS(30.0);
        simulationCameraProperties.setAvgLatencyMs(35.0);
        simulationCameraProperties.setLatencyStdDevMs(5.0);

        return simulationCameraProperties;
    }
}