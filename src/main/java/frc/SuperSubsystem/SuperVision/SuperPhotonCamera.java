// SuperPhotonCamera.java
package frc.SuperSubsystem.SuperVision;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.SuperSubsystem.SuperVision.VisionEntrys.FrameSnapshot;
import frc.SuperSubsystem.SuperVision.VisionEntrys.VisionFrameMetadata;

/**
 * SuperPhotonCamera
 *
 * Qué es:
 * Wrapper de alto nivel para PhotonVision que estandariza cómo se consumen los frames de visión
 * desde el roboRIO.
 *
 * Para qué sirve:
 * - Leer frames de PhotonVision sin perder datos por mal uso de la API.
 * - Mantener los datos de cada frame juntos (timestamp, targets, best target, pose estimada).
 * - Opcionalmente calcular pose del robot usando AprilTags.
 * - Guardar un historial corto de frames recientes para debug y decisiones del robot.
 *
 * Cómo se usa en el robot:
 * 1) Construyes la cámara con el nombre configurado en PhotonVision y el transform robot->cámara.
 * 2) En tu periodic() llamas update() exactamente una vez por loop.
 * 3) Consumes datos con getLatestFrame(), getLatestBestTargetMetadata() o estimate...().
 * 4) Si haces odometría con visión, usa updateWithPoseConsumer(...) y ahí mismo llama addVisionMeasurement(...)
 *    con el timestamp correcto.
 *
 * Regla crítica:
 * - getAllUnreadResults() se debe llamar exactamente una vez por loop. Esta clase lo hace por ti.
 *   No lo llames en otro lugar o vas a vaciar la FIFO y perder frames.
 *
 * Notas de diseño:
 * - Se usa un ring buffer acotado para no crecer memoria.
 * - Las listas se copian para evitar mutaciones accidentales.
 * - La pose estimada se calcula por frame (si está habilitada) y puede entregarse inmediatamente a un consumidor.
 */
public class SuperPhotonCamera {

    /**
     * Nombre de la cámara tal como aparece en PhotonVision UI.
     * Este nombre debe coincidir exactamente con el configurado en el coprocessor.
     */
    private final String cameraName;

    /**
     * Transformación robot -> cámara.
     * Debe representar la posición y orientación de la cámara con respecto al origen del robot.
     * PhotonPoseEstimator espera este transform.
     */
    private final Transform3d robotToCameraTransform3d;

    /**
     * Transformación cámara -> robot.
     * Es la inversa de robotToCameraTransform3d.
     * Se usa en ciertos cálculos de PhotonUtils.
     */
    private final Transform3d cameraToRobotTransform3d;

    /**
     * Instancia PhotonCamera que habla por NetworkTables con PhotonVision.
     */
    private final PhotonCamera photonCamera;

    /**
     * Buffer acotado con snapshots recientes.
     * Permite inspeccionar frames recientes y evita mezclar datos entre frames.
     */
    private final Deque<FrameSnapshot> frameRingBuffer;

    /**
     * Máximo de snapshots almacenados.
     * Si el buffer excede este tamaño, se descartan los más viejos.
     */
    private final int maxBufferedFrames;

    /**
     * Pose estimator para AprilTags.
     * Null significa que la estimación de pose está deshabilitada.
     */
    private PhotonPoseEstimator photonPoseEstimator;

    /**
     * Modo seleccionado para estimar pose cuando photonPoseEstimator está habilitado.
     */
    private PoseEstimationModeEnum poseEstimationMode;

    /**
     * Pose de referencia usada en modos de estimación basados en referencia.
     * Ejemplo: CLOSEST_TO_REFERENCE_POSE.
     */
    private Pose3d referencePoseForEstimation;

    /**
     * Default recomendado si quieres tener historial para debug sin gastar memoria.
     * 60 frames a 30 FPS es ~2 segundos de historia.
     */
    private static final double DEFAULT_MAX_BUFFERED_FRAMES = 60;

    /**
     * Transform por default si aún no tienes medidas reales.
     * En robot real debes reemplazarlo por valores medidos.
     */
    private static final Transform3d DEFAULT_ROBOT_TO_CAMERA_TRANSFORM3D = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    /**
     * Nombre por default si aún no has nombrado la cámara en PhotonVision.
     * Es común que el nombre real sea algo como "FrontCam" o "TagCam".
     */
    private static final String DEFAULT_CAMERA_NAME = "PhotonVision";

    /**
     * Constructor default.
     * Usa nombre y transform default.
     * Útil para prototipos, no recomendado para robot final sin configurar transform.
     */
    public SuperPhotonCamera() {
        this(
                DEFAULT_CAMERA_NAME,
                DEFAULT_ROBOT_TO_CAMERA_TRANSFORM3D,
                (int) DEFAULT_MAX_BUFFERED_FRAMES
        );
    }

    /**
     * Constructor con nombre de cámara.
     * Usa transform default.
     *
     * @param cameraName nombre de la cámara en PhotonVision UI
     */
    public SuperPhotonCamera(String cameraName) {
        this(
                cameraName,
                DEFAULT_ROBOT_TO_CAMERA_TRANSFORM3D,
                (int) DEFAULT_MAX_BUFFERED_FRAMES
        );
    }

    /**
     * Constructor con nombre y transform robot->cámara.
     * Usa buffer default.
     *
     * @param cameraName nombre de la cámara en PhotonVision UI
     * @param robotToCameraTransform3d transform robot->cámara medido en el robot
     */
    public SuperPhotonCamera(String cameraName, Transform3d robotToCameraTransform3d) {
        this(
                cameraName,
                robotToCameraTransform3d,
                (int) DEFAULT_MAX_BUFFERED_FRAMES
        );
    }

    /**
     * Constructor completo.
     *
     * Uso típico:
     * - cameraName: "FrontTagCam"
     * - robotToCameraTransform3d: transform medido robot->cámara
     * - maxBufferedFrames: 30 a 120 según tu preferencia
     *
     * @param cameraName nombre de la cámara configurado en PhotonVision UI
     * @param robotToCameraTransform3d transform robot->cámara (posición y orientación)
     * @param maxBufferedFrames cantidad máxima de snapshots a guardar
     */
    public SuperPhotonCamera(String cameraName, Transform3d robotToCameraTransform3d, int maxBufferedFrames) {
        this.cameraName = cameraName;
        this.robotToCameraTransform3d = robotToCameraTransform3d;
        this.cameraToRobotTransform3d = robotToCameraTransform3d.inverse();

        this.photonCamera = new PhotonCamera(cameraName);

        this.maxBufferedFrames = Math.max(1, maxBufferedFrames);
        this.frameRingBuffer = new ArrayDeque<>(this.maxBufferedFrames);

        this.photonPoseEstimator = null;
        this.poseEstimationMode = PoseEstimationModeEnum.COPROCESSOR_MULTI_TAG;
        this.referencePoseForEstimation = new Pose3d();
    }

    /**
     * Habilita estimación de pose con AprilTags.
     *
     * Requisito:
     * - Debes tener un AprilTagFieldLayout válido del juego actual.
     *
     * Resultado:
     * - estimateRobotPose(...) empezará a devolver poses cuando sea posible.
     *
     * @param aprilTagFieldLayout layout de AprilTags del campo
     */
    public void enableAprilTagPoseEstimation(AprilTagFieldLayout aprilTagFieldLayout) {
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCameraTransform3d);
    }

    /**
     * Define el modo de estimación de pose.
     *
     * Cuándo usar cada uno:
     * - COPROCESSOR_MULTI_TAG: recomendado cuando PhotonVision está configurado para multi-tag
     * - LOWEST_AMBIGUITY: útil cuando normalmente ves un solo tag
     * - CLOSEST_TO_REFERENCE_POSE: útil si tu odometría es buena y quieres estabilidad
     * - AVERAGE_BEST_TARGETS: útil como smoothing en algunas geometrías
     *
     * @param mode modo seleccionado
     */
    public void setPoseEstimationMode(PoseEstimationModeEnum mode) {
        this.poseEstimationMode = mode;
    }

    /**
     * Define la pose de referencia usada por modos basados en referencia.
     *
     * Uso típico:
     * - Llamar esto cada loop con tu pose estimada actual del drivetrain.
     * - Convertir Pose2d a Pose3d si tu estimador principal está en 2D.
     *
     * Si no actualizas esto, el modo CLOSEST_TO_REFERENCE_POSE puede elegir soluciones equivocadas.
     *
     * @param referencePose pose de referencia field->robot
     */
    public void setReferencePoseForEstimation(Pose3d referencePose) {
        this.referencePoseForEstimation = referencePose;
    }

    /**
     * Actualiza el estado interno de la cámara.
     *
     * Regla:
     * Debes llamarlo exactamente una vez por robot loop.
     *
     * Efecto:
     * - Drena todos los frames no leídos desde PhotonVision
     * - Crea FrameSnapshot por cada frame
     * - Opcionalmente calcula pose por frame (si está habilitado)
     * - Mete snapshots al buffer acotado
     */
    public void update() {
        updateWithPoseConsumer(null);
    }

    /**
     * Actualiza la cámara y opcionalmente procesa poses inmediatamente.
     *
     * Uso recomendado:
     * Si tu drivetrain tiene un pose estimator (por ejemplo SwerveDrivePoseEstimator),
     * pasa un Consumer que haga addVisionMeasurement usando:
     * - el pose estimate
     * - el timestamp del estimate
     * - y std devs definidos por tu equipo
     *
     * Regla:
     * No llames getAllUnreadResults() en ningún otro lugar si usas este wrapper.
     *
     * @param estimatedRobotPoseConsumer consumidor opcional de EstimatedRobotPose; puede ser null
     */
    @SuppressWarnings("null")
    public void updateWithPoseConsumer(Consumer<EstimatedRobotPose> estimatedRobotPoseConsumer) {

        List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        if (unreadResults == null || unreadResults.isEmpty()) {
            return;
        }

        for (PhotonPipelineResult pipelineResult : unreadResults) {
            if (pipelineResult == null) {
                continue;
            }

            List<PhotonTrackedTarget> targetsFromResult = pipelineResult.getTargets();
            List<PhotonTrackedTarget> trackedTargets =
                    (targetsFromResult != null) ? List.copyOf(targetsFromResult) : Collections.emptyList();

            Optional<VisionFrameMetadata> bestTargetMetadata = Optional.empty();
            if (pipelineResult.hasTargets()) {
                PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
                if (bestTarget != null) {
                    bestTargetMetadata = Optional.of(
                            new VisionFrameMetadata(pipelineResult.getTimestampSeconds(), bestTarget)
                    );
                }
            }

            Optional<EstimatedRobotPose> estimatedRobotPose = estimateRobotPose(pipelineResult);

            if (estimatedRobotPoseConsumer != null) {
                estimatedRobotPose.ifPresent(estimatedRobotPoseConsumer);
            }

            FrameSnapshot snapshot = new FrameSnapshot(
                    pipelineResult,
                    trackedTargets,
                    bestTargetMetadata,
                    estimatedRobotPose
            );

            frameRingBuffer.addLast(snapshot);

            while (frameRingBuffer.size() > maxBufferedFrames) {
                frameRingBuffer.removeFirst();
            }
        }
    }

    /**
     * Estima la pose del robot a partir del pipeline result y el modo configurado.
     *
     * Si la estimación no está habilitada o no se puede resolver, devuelve Optional.empty().
     *
     * Nota:
     * Este método no filtra por calidad. El filtro de stdevs, ambigüedad o edad del frame
     * normalmente se hace donde consumes la pose (en el drivetrain).
     *
     * @param pipelineResult pipeline result del frame
     * @return Optional con EstimatedRobotPose si hay solución
     */
    private Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult pipelineResult) {
        if (photonPoseEstimator == null) {
            return Optional.empty();
        }

        return switch (poseEstimationMode) {
            case COPROCESSOR_MULTI_TAG ->
                    photonPoseEstimator.estimateCoprocMultiTagPose(pipelineResult);

            case LOWEST_AMBIGUITY ->
                    photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);

            case CLOSEST_TO_REFERENCE_POSE ->
                    photonPoseEstimator.estimateClosestToReferencePose(pipelineResult, referencePoseForEstimation);

            case AVERAGE_BEST_TARGETS ->
                    photonPoseEstimator.estimateAverageBestTargetsPose(pipelineResult);

            default ->
                    Optional.empty();
        };
    }

    /**
     * Devuelve una copia inmutable del buffer de snapshots.
     *
     * Esto asigna memoria al crear una nueva lista.
     * Para uso típico de robot, es mejor getLatestFrame().
     *
     * @return lista con snapshots de más viejo a más nuevo
     */
    public List<FrameSnapshot> getBufferedFrames() {
        return List.copyOf(frameRingBuffer);
    }

    /**
     * Devuelve el snapshot más reciente disponible.
     *
     * @return Optional con el snapshot más reciente
     */
    public Optional<FrameSnapshot> getLatestFrame() {
        return frameRingBuffer.isEmpty() ? Optional.empty() : Optional.of(frameRingBuffer.getLast());
    }

    /**
     * Devuelve metadata del best target del último frame.
     *
     * Útil para aim simple:
     * - yawDegrees
     * - pitchDegrees
     * - area
     *
     * @return Optional con metadata del best target
     */
    public Optional<VisionFrameMetadata> getLatestBestTargetMetadata() {
        return getLatestFrame().flatMap(frame -> frame.bestTargetMetadata);
    }

    /**
     * Checa si los datos más recientes están viejos.
     *
     * Esto es útil para:
     * - deshabilitar auto-aim si ya no hay visión reciente
     * - ignorar pose measurements viejas
     *
     * @param maxAgeSeconds edad máxima aceptable en segundos
     * @return true si no hay frames o si el último frame excede maxAgeSeconds
     */
    public boolean isVisionStale(double maxAgeSeconds) {
        return getLatestFrame()
                .map(frame -> frame.getFrameAgeSeconds() > maxAgeSeconds)
                .orElse(true);
    }

    /**
     * Indica si PhotonVision reporta que la cámara está conectada.
     *
     * @return true si hay conexión, false si no
     */
    public boolean isConnected() {
        return photonCamera.isConnected();
    }

    /**
     * Activa o desactiva el driver mode.
     *
     * Driver mode normalmente hace:
     * - baja latencia
     * - apaga procesamiento de visión
     * - útil para view de driver
     *
     * @param driverModeEnabled true para activar driver mode
     */
    public void setDriverMode(boolean driverModeEnabled) {
        photonCamera.setDriverMode(driverModeEnabled);
    }

    /**
     * Devuelve si driver mode está activo.
     *
     * @return true si driver mode está activo
     */
    public boolean getDriverMode() {
        return photonCamera.getDriverMode();
    }

    /**
     * Estima la pose field->robot usando solamente el best target y PhotonUtils.
     *
     * Esto es un método de conveniencia, no el camino principal para pose estimation robusta.
     * Es útil como:
     * - fallback
     * - debug
     * - pruebas rápidas
     *
     * Requisitos:
     * - el best target debe ser un AprilTag válido
     * - el id debe existir en el AprilTagFieldLayout
     *
     * @param aprilTagFieldLayout layout del campo
     * @return Optional con pose field->robot estimada
     */
    public Optional<Pose3d> estimateFieldToRobotFromBestAprilTag(AprilTagFieldLayout aprilTagFieldLayout) {

        Optional<VisionFrameMetadata> bestMetadataOptional = getLatestBestTargetMetadata();
        if (bestMetadataOptional.isEmpty()) {
            return Optional.empty();
        }

        VisionFrameMetadata bestMetadata = bestMetadataOptional.get();
        int fiducialId = bestMetadata.fiducialId;

        Optional<Pose3d> fieldToTagPose = aprilTagFieldLayout.getTagPose(fiducialId);
        if (fieldToTagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d fieldToRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestMetadata.bestCameraToTarget,
                fieldToTagPose.get(),
                cameraToRobotTransform3d
        );

        return Optional.of(fieldToRobotPose);
    }

    /**
     * Estima la traslación 2D cámara->target si ya tienes una distancia.
     *
     * Uso típico:
     * - tienes un range estimado por trig con pitch o por sensor externo
     * - quieres una traslación 2D para apuntar o alinear
     *
     * Nota:
     * El signo del yaw depende de tu convención.
     * Si el robot gira al revés, revisa el signo de Rotation2d.
     *
     * @param distanceMeters distancia estimada a target en metros
     * @return Optional con Translation2d cámara->target
     */
    public Optional<Translation2d> estimateCameraToTargetTranslationMeters(double distanceMeters) {
        Optional<VisionFrameMetadata> bestMetadataOptional = getLatestBestTargetMetadata();
        if (bestMetadataOptional.isEmpty()) {
            return Optional.empty();
        }

        double yawDegrees = bestMetadataOptional.get().yawDegrees;
        return Optional.of(
                PhotonUtils.estimateCameraToTargetTranslation(distanceMeters, Rotation2d.fromDegrees(-yawDegrees))
        );
    }

    /**
     * Devuelve el nombre configurado de la cámara.
     *
     * @return nombre de la cámara
     */
    public String getName() {
        return cameraName;
    }

    /**
     * Devuelve el transform robot->cámara.
     *
     * @return transform robot->cámara
     */
    public Transform3d getRobotToCameraTransform3d() {
        return robotToCameraTransform3d;
    }

    /**
     * Devuelve el transform cámara->robot.
     *
     * @return transform cámara->robot
     */
    public Transform3d getCameraToRobotTransform3d() {
        return cameraToRobotTransform3d;
    }
}
