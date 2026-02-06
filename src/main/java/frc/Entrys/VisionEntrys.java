// VisionEntrys.java
package frc.Entrys;

import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.Enums.PoseEstimateSourceEnum;
import frc.SuperSubsystem.SuperVision.SuperPhotonCamera;

/**
 * VisionEntrys
 *
 * Este archivo agrupa varias estructuras de datos que se usan en el sistema de visión.
 *
 * Qué contiene:
 * - VisionFrameMetadata: metadata inmutable de un target observado en un frame.
 * - FrameSnapshot: snapshot inmutable de un frame completo (targets, best target, pose estimada, timestamp).
 * - CameraEntry: empareja una SuperPhotonCamera con un nivel de ruido (std devs).
 * - CameraSpecs: define especificaciones de una cámara (nombre, transform, std devs) para construirla en otro lugar.
 *
 * Cómo se usa normalmente:
 * - SuperPhotonCamera genera FrameSnapshot y los guarda en su ring buffer.
 * - El drivetrain o un VisionSubsystem mantiene una lista de CameraEntry para tener:
 *   - la cámara
 *   - su std dev asociado para addVisionMeasurement(...)
 */
public class VisionEntrys {

    /**
     * VisionFrameMetadata
     *
     * Metadata inmutable asociada a un target detectado por PhotonVision.
     * Idealmente se usa para el "best target" del frame.
     */
    public static final class VisionFrameMetadata {

        public final double timestampSeconds;

        public final double area;
        public final double pitchDegrees;
        public final double yawDegrees;
        public final double skewDegrees;
        public final double poseAmbiguity;

        public final Transform3d bestCameraToTarget;
        public final Transform3d alternateCameraToTarget;

        public final List<TargetCorner> detectedCorners;
        public final List<TargetCorner> minAreaRectCorners;

        public final int objectDetectedClassId;
        public final int fiducialId;

        /**
         * Construye metadata a partir de un PhotonTrackedTarget.
         *
         * @param timestampSeconds timestamp del frame en el que se observó el target
         * @param trackedTarget target reportado por PhotonVision
         */
        public VisionFrameMetadata(double timestampSeconds, PhotonTrackedTarget trackedTarget) {
            this.timestampSeconds = timestampSeconds;

            this.area = trackedTarget.getArea();
            this.pitchDegrees = trackedTarget.getPitch();
            this.yawDegrees = trackedTarget.getYaw();
            this.skewDegrees = trackedTarget.getSkew();
            this.poseAmbiguity = trackedTarget.getPoseAmbiguity();

            this.bestCameraToTarget = trackedTarget.getBestCameraToTarget();
            this.alternateCameraToTarget = trackedTarget.getAlternateCameraToTarget();

            List<TargetCorner> corners = trackedTarget.getDetectedCorners();
            this.detectedCorners = (corners != null) ? List.copyOf(corners) : Collections.emptyList();

            List<TargetCorner> minRectCorners = trackedTarget.getMinAreaRectCorners();
            this.minAreaRectCorners = (minRectCorners != null) ? List.copyOf(minRectCorners) : Collections.emptyList();

            this.objectDetectedClassId = trackedTarget.getDetectedObjectClassID();
            this.fiducialId = trackedTarget.getFiducialId();
        }
    }

    /**
     * FrameSnapshot
     *
     * Snapshot inmutable de un frame de cámara.
     *
     * Para qué sirve:
     * - Mantener unidos timestamp, targets y pose estimada
     * - Evitar usar datos mezclados entre frames
     * - Tener historial para debug
     */
    public static final class FrameSnapshot {

        public final PhotonPipelineResult pipelineResult;
        public final double timestampSeconds;
        public final List<PhotonTrackedTarget> trackedTargets;
        public final Optional<VisionFrameMetadata> bestTargetMetadata;
        public final Optional<EstimatedRobotPose> estimatedRobotPose;

        /**
         * Construye un snapshot de frame.
         *
         * @param pipelineResult pipeline result del frame
         * @param trackedTargets lista inmutable de targets detectados
         * @param bestTargetMetadata metadata del best target si existe
         * @param estimatedRobotPose pose estimada si existe
         */
        public FrameSnapshot(
                PhotonPipelineResult pipelineResult,
                List<PhotonTrackedTarget> trackedTargets,
                Optional<VisionFrameMetadata> bestTargetMetadata,
                Optional<EstimatedRobotPose> estimatedRobotPose
        ) {
            this.pipelineResult = pipelineResult;
            this.timestampSeconds = pipelineResult.getTimestampSeconds();
            this.trackedTargets = trackedTargets;
            this.bestTargetMetadata = bestTargetMetadata;
            this.estimatedRobotPose = estimatedRobotPose;
        }

        /**
         * Edad del frame en segundos comparado con el tiempo actual del roboRIO.
         *
         * Esto se usa para descartar datos viejos.
         *
         * @return edad del frame en segundos
         */
        public double getFrameAgeSeconds() {
            return Timer.getFPGATimestamp() - timestampSeconds;
        }
    }

    /**
     * CameraEntry
     *
     * Relaciona una cámara con el nivel de ruido que se usará al inyectar visión en odometría.
     *
     * Uso típico:
     * - VisionSubsystem tiene List<CameraEntry>
     * - Por cada cámara llama camera.updateWithPoseConsumer(...)
     * - Si hay pose, la mete al pose estimator con StdDevs.getStandardDeviations()
     */
    public static final class CameraEntry {

        public final SuperPhotonCamera camera;
        public final PoseEstimateSourceEnum standardDeviationsLevel;

        /**
         * Construye un entry para una cámara.
         *
         * @param camera instancia de SuperPhotonCamera
         * @param standardDeviationsLevel nivel de desviaciones estándar para esa cámara
         */
        public CameraEntry(SuperPhotonCamera camera, PoseEstimateSourceEnum standardDeviationsLevel) {
            this.standardDeviationsLevel = Objects.requireNonNull(standardDeviationsLevel);
            this.camera = Objects.requireNonNull(camera);
        }
    }

    /**
     * CameraSpecs
     *
     * Especificación declarativa de una cámara.
     *
     * Para qué sirve:
     * - Definir en un solo lugar nombre + transform + std devs
     * - Luego construir SuperPhotonCamera en otro subsistema usando estos specs
     */
    public static final class CameraSpecs {

        public final String cameraName;
        public final Transform3d cameraToRobotTransform3d;
        public final PoseEstimateSourceEnum standardDeviationsLevel;

        /**
         * Construye specs de cámara.
         *
         * @param cameraName nombre de la cámara en PhotonVision UI
         * @param cameraToRobotTransform3d transform cámara->robot
         * @param standardDeviationsLevel std devs a usar para esa cámara
         */
        public CameraSpecs(
                String cameraName,
                Transform3d cameraToRobotTransform3d,
                PoseEstimateSourceEnum standardDeviationsLevel
        ) {
            this.cameraName = cameraName;
            this.cameraToRobotTransform3d = cameraToRobotTransform3d;
            this.standardDeviationsLevel = standardDeviationsLevel;
        }

        /**
         * Construye una SuperPhotonCamera usando estos specs.
         *
         * Nota:
         * SuperPhotonCamera recibe robot->cámara, pero aquí guardamos cámara->robot.
         * Por eso se invierte el transform.
         *
         * @return una nueva instancia de SuperPhotonCamera configurada con estos specs
         */
        public SuperPhotonCamera buildCamera() {
            Transform3d robotToCameraTransform3d = cameraToRobotTransform3d.inverse();
            return new SuperPhotonCamera(cameraName, robotToCameraTransform3d);
        }
    }
}
