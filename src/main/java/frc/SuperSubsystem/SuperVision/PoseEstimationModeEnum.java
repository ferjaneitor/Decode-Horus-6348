// PoseEstimationModeEnum.java
package frc.SuperSubsystem.SuperVision;

/**
 * PoseEstimationModeEnum
 *
 * Define el método que se usará para estimar pose a partir de un PhotonPipelineResult.
 *
 * Este enum se usa dentro de SuperPhotonCamera para seleccionar qué método llamar del PhotonPoseEstimator.
 *
 * Recomendación general:
 * - COPROCESSOR_MULTI_TAG suele ser el mejor si tienes multi-tag estable.
 * - CLOSEST_TO_REFERENCE_POSE es muy estable si tu pose de referencia está bien actualizada.
 */
public enum PoseEstimationModeEnum {

    /**
     * Estimación multi-tag realizada por el coprocessor.
     * Suele ser la opción más robusta cuando hay múltiples tags visibles.
     */
    COPROCESSOR_MULTI_TAG,

    /**
     * Selecciona la solución con menor ambigüedad.
     * Útil en escenarios de 1 tag y soluciones que a veces "saltan".
     */
    LOWEST_AMBIGUITY,

    /**
     * Selecciona la solución más cercana a la pose de referencia provista por el robot.
     * Requiere que la referencia se actualice frecuentemente (idealmente cada loop).
     */
    CLOSEST_TO_REFERENCE_POSE,

    /**
     * Promedia soluciones basadas en los mejores targets.
     * Puede suavizar ruido pero también puede introducir sesgo según geometría.
     */
    AVERAGE_BEST_TARGETS
}
