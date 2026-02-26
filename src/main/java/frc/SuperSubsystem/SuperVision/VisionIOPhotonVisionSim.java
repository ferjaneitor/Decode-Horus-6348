// File: src/main/java/frc/SuperSubsystem/SuperVision/VisionIOPhotonVisionSim.java
package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

    private static VisionSystemSim sharedVisionSystemSimulation;

    @SuppressWarnings("FieldCanBeLocal")
    private final PhotonCameraSim photonCameraSimulation;

    public VisionIOPhotonVisionSim(
            String cameraName,
            Transform3d robotToCameraTransform3d,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {
        super(cameraName, robotToCameraTransform3d, aprilTagFieldLayout);

        if (sharedVisionSystemSimulation == null) {
            sharedVisionSystemSimulation = new VisionSystemSim("main");
            sharedVisionSystemSimulation.addAprilTags(aprilTagFieldLayout);
        }

        SimCameraProperties simulationCameraProperties = createReasonableSimulationCameraProperties();

        photonCameraSimulation =
                new PhotonCameraSim(this.photonCamera, simulationCameraProperties, aprilTagFieldLayout);

        // Keep simulation performance sane (streams/wireframe can destroy loop time)
        photonCameraSimulation.enableRawStream(false);
        photonCameraSimulation.enableProcessedStream(false);
        photonCameraSimulation.enableDrawWireframe(false);

        // Optional: make detection slightly more realistic / stable
        photonCameraSimulation.setMinTargetAreaPixels(120.0);
        photonCameraSimulation.setMaxSightRange(8.0);

        sharedVisionSystemSimulation.addCamera(photonCameraSimulation, robotToCameraTransform3d);
    }

    @Override
    public void updateInputs(VisionIOInputs visionInputs) {
        super.updateInputs(visionInputs);

        // In desktop simulation there is no real PhotonVision heartbeat;
        // treat this camera as connected if we are simulating.
        if (RobotBase.isSimulation()) {
            visionInputs.cameraConnected = true;
        }
    }

    /** Call AFTER your physics step (MapleSim step) each sim tick. */
    public static void updateVisionSystemSimulation(Pose2d simulatedRobotPoseMeters) {
        if (sharedVisionSystemSimulation != null) {
            sharedVisionSystemSimulation.update(simulatedRobotPoseMeters);
        }
    }

    /** Call when resetting simulation state. */
    public static void resetVisionSystemSimulation(Pose2d simulatedRobotPoseMeters) {
        if (sharedVisionSystemSimulation != null) {
            sharedVisionSystemSimulation.resetRobotPose(simulatedRobotPoseMeters);
        }
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

        // Optional: small calibration imperfections (helps avoid “too perfect” sim)
        simulationCameraProperties.setCalibError(0.25, 0.08);

        return simulationCameraProperties;
    }
}