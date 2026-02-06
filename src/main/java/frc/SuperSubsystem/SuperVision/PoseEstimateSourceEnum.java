// PoseEstimateSourceEnum.java
package frc.SuperSubsystem.SuperVision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * PoseEstimateSourceEnum
 *
 * Define niveles de confianza (ruido) para mediciones de visión.
 *
 * Para qué sirve:
 * - Representa los std devs (desviación estándar) para x, y y theta
 * - Se usa típicamente al llamar a addVisionMeasurement(...) en un PoseEstimator de WPILib
 *
 * Cómo se usa:
 * - LOW: medición muy confiable, std dev pequeño
 * - MEDIUM: medición normal
 * - HIGH: medición ruidosa o condiciones malas
 * - NONE: casi inutilizable, se usa para ignorar mediciones
 *
 * Nota:
 * - Estos valores son ejemplos. Cada equipo debe tunearlos en cancha.
 */
public enum PoseEstimateSourceEnum {

    /**
     * Alta confianza en visión.
     * Útil cuando tienes multi-tag estable y buena calibración.
     */
    LOW(0.025),

    /**
     * Confianza media.
     * Útil como default seguro si no has tuneado.
     */
    MEDIUM(0.15),

    /**
     * Baja confianza.
     * Útil cuando hay pocas detecciones o mala geometría.
     */
    HIGH(0.3),

    /**
     * Medición esencialmente inútil.
     * Se puede usar para representar "no usar visión".
     */
    NONE(99.0);

    /**
     * Matriz de std devs para x, y, theta.
     * Esta es la forma que WPILib espera para setVisionMeasurementStdDevs(...) o para cálculos equivalentes.
     */
    private final Matrix<N3, N1> standardDeviations;

    /**
     * Constructor con una sola magnitud para x, y y theta.
     *
     * @param deviation valor usado para x, y y theta
     */
    PoseEstimateSourceEnum(double deviation) {
        this(deviation, deviation, deviation);
    }

    /**
     * Constructor con std devs independientes por eje.
     *
     * @param xDeviation desviación estándar para X
     * @param yDeviation desviación estándar para Y
     * @param thetaDeviation desviación estándar para theta
     */
    PoseEstimateSourceEnum(double xDeviation, double yDeviation, double thetaDeviation) {
        this.standardDeviations = MatBuilder.fill(Nat.N3(), Nat.N1(), xDeviation, yDeviation, thetaDeviation);
    }

    /**
     * Devuelve la matriz de desviaciones estándar asociada al nivel.
     *
     * @return matriz 3x1 con std devs para x, y, theta
     */
    public Matrix<N3, N1> getStandardDeviations() {
        return standardDeviations;
    }
}
