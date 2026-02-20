// File: src/main/java/frc/robot/Climber/ClimberSubsystem.java
package frc.robot.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.AutoLogger.ClimberIOInputsAutoLogged;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Climber.IO.ClimberIO;
import frc.robot.Climber.IO.ClimberIOSpark;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public final class ClimberSubsystem extends SubsystemBase {

    // Tolerance-only state detection (no hysteresis)
    private static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS = 0.05;
    private static final double CLIMBER_RETRACTED_TARGET_ROTATIONS = 0.0;

    private final ClimberIO climberHardwareInterface;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean isFullyExtended = false;
    private boolean isFullyRetracted = true;

    private double lastRequestedClimberTargetRotations = 0.0;

    private static final double OPEN_LOOP_VOLTAGE_VOLTS =
            Constants.BATTERY_VOLTAGE * ClimberConstants.CLIMBER_MAX_DUTY_CYCLE;

    /**
     * Convenience constructor that builds Spark-based IO (same as your original subsystem did).
     */
    public ClimberSubsystem() {
        this(
                new ClimberIOSpark(
                        new SuperSparkMax(
                                ClimberConstants.LEFT_CLIMBER_MOTOR_ID,
                                SparkLowLevel.MotorType.kBrushless,
                                ClimberConstants.CLIMBER_MOTOR_CONFIG()
                        ),
                        new SuperSparkMax(
                                ClimberConstants.RIGHT_CLIMBER_MOTOR_ID,
                                SparkLowLevel.MotorType.kBrushless,
                                ClimberConstants.CLIMBER_MOTOR_CONFIG()
                        )
                )
        );
    }

    public ClimberSubsystem(ClimberIO climberHardwareInterface) {
        this.climberHardwareInterface = climberHardwareInterface;
        this.lastRequestedClimberTargetRotations = CLIMBER_RETRACTED_TARGET_ROTATIONS;
    }

    @Override
    public void periodic() {
        climberHardwareInterface.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        updateExtensionStateFromPosition();
    }

    private void updateExtensionStateFromPosition() {
        double leftPositionRotations = inputs.leftClimberPositionRotations;
        double rightPositionRotations = inputs.rightClimberPositionRotations;
        double averagePositionRotations = (leftPositionRotations + rightPositionRotations) * 0.5;

        double extendedTargetRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION;
        double retractedTargetRotations = CLIMBER_RETRACTED_TARGET_ROTATIONS;

        boolean isAtExtended =
                averagePositionRotations >= (extendedTargetRotations - CLIMBER_POSITION_TOLERANCE_ROTATIONS);

        boolean isAtRetracted =
                averagePositionRotations <= (retractedTargetRotations + CLIMBER_POSITION_TOLERANCE_ROTATIONS);

        // Resolve state with tolerance only (no hysteresis).
        // If both are true (overlap due to config/tolerance), choose the closer target.
        if (isAtExtended && !isAtRetracted) {
            isFullyExtended = true;
            isFullyRetracted = false;
            return;
        }

        if (isAtRetracted && !isAtExtended) {
            isFullyRetracted = true;
            isFullyExtended = false;
            return;
        }

        if (isAtExtended && isAtRetracted) {
            double distanceToExtended = Math.abs(extendedTargetRotations - averagePositionRotations);
            double distanceToRetracted = Math.abs(averagePositionRotations - retractedTargetRotations);

            boolean shouldPreferExtended = distanceToExtended <= distanceToRetracted;

            isFullyExtended = shouldPreferExtended;
            isFullyRetracted = !shouldPreferExtended;
            return;
        }

        // In between: neither fully extended nor fully retracted
        isFullyExtended = false;
        isFullyRetracted = false;
    }

    // ---------------- Manual open-loop control ----------------

    public void expand() {
        climberHardwareInterface.setClimberVoltage(OPEN_LOOP_VOLTAGE_VOLTS);
    }

    public void retract() {
        climberHardwareInterface.setClimberVoltage(-OPEN_LOOP_VOLTAGE_VOLTS);
    }

    public void stop() {
        climberHardwareInterface.stopClimber();
    }

    // ---------------- Auto (PID simple, like Intake) ----------------

    private void autoControlMotors(double desiredPositionRotations) {
        lastRequestedClimberTargetRotations = desiredPositionRotations;
        climberHardwareInterface.setClimberPositionPidRotations(lastRequestedClimberTargetRotations);
    }

    public void AutoExpand() {
        if (!isFullyExtended) {
            autoControlMotors(ClimberConstants.CLIMBER_EXTENDED_POSITION);
        }
    }

    public void AutoRetract() {
        if (!isFullyRetracted) {
            autoControlMotors(CLIMBER_RETRACTED_TARGET_ROTATIONS);
        }
    }

    // ---------------- State ----------------

    public double getLeftClimberPositionRotations() {
        return inputs.leftClimberPositionRotations;
    }

    public double getRightClimberPositionRotations() {
        return inputs.rightClimberPositionRotations;
    }

    public double getAverageClimberPositionRotations() {
        return (inputs.leftClimberPositionRotations + inputs.rightClimberPositionRotations) * 0.5;
    }

    public boolean isClimberExtended() {
        return isFullyExtended;
    }

    public boolean isClimberRetracted() {
        return isFullyRetracted;
    }
}
