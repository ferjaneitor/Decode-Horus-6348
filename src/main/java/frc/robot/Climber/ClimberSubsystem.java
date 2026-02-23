package frc.robot.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.AutoLogger.ClimberIOInputsAutoLogged;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Climber.IO.ClimberIO;
import frc.robot.Climber.IO.ClimberIOSpark;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public final class ClimberSubsystem extends SubsystemBase {

    private static final double CLIMBER_RETRACTED_TARGET_ROTATIONS = 0.0;

    private static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS = 0.05;
    private static final double CLIMBER_POSITION_HYSTERESIS_ROTATIONS = 0.05;

    private static final double CLIMBER_SOFT_LIMIT_MINIMUM_ROTATIONS = -0.10;
    private static final double CLIMBER_SOFT_LIMIT_MAXIMUM_ROTATIONS = ClimberConstants.CLIMBER_EXTENDED_POSITION + 0.10;

    private final ClimberIO climberHardwareInterface;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean isFullyExtended = false;
    private boolean isFullyRetracted = true;

    private double lastRequestedClimberTargetRotations = CLIMBER_RETRACTED_TARGET_ROTATIONS;

    private static final double OPEN_LOOP_VOLTAGE_VOLTS =
            Constants.BATTERY_VOLTAGE * ClimberConstants.CLIMBER_MAX_DUTY_CYCLE;

    private boolean didBootInitialization = false;

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
    }

    @Override
    public void periodic() {
        climberHardwareInterface.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (!didBootInitialization) {
            performBootInitialization();
            didBootInitialization = true;
        }

        if (DriverStation.isDisabled()) {
            climberHardwareInterface.stopClimber();
            return;
        }

        updateExtensionStateFromPosition();
    }

    private void performBootInitialization() {
        // Dejamos el encoder “cero” en retract (como intake).
        // En real, asume que arrancas retracted.
        climberHardwareInterface.setClimberRawEncoderPositionRotations(0.0, 0.0);
        lastRequestedClimberTargetRotations = CLIMBER_RETRACTED_TARGET_ROTATIONS;
    }

    private void updateExtensionStateFromPosition() {
        double averagePositionRotations = getAverageClimberPositionRotations();

        double extendedTargetRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION;
        double retractedTargetRotations = CLIMBER_RETRACTED_TARGET_ROTATIONS;

        boolean isAtExtended = averagePositionRotations >= (extendedTargetRotations - CLIMBER_POSITION_TOLERANCE_ROTATIONS);
        boolean isAtRetracted = averagePositionRotations <= (retractedTargetRotations + CLIMBER_POSITION_TOLERANCE_ROTATIONS);

        if (isAtExtended) {
            isFullyExtended = true;
            isFullyRetracted = false;
        } else if (averagePositionRotations < (extendedTargetRotations - CLIMBER_POSITION_TOLERANCE_ROTATIONS - CLIMBER_POSITION_HYSTERESIS_ROTATIONS)) {
            isFullyExtended = false;
        }

        if (isAtRetracted) {
            isFullyRetracted = true;
            isFullyExtended = false;
        } else if (averagePositionRotations > (retractedTargetRotations + CLIMBER_POSITION_TOLERANCE_ROTATIONS + CLIMBER_POSITION_HYSTERESIS_ROTATIONS)) {
            isFullyRetracted = false;
        }
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

    // ---------------- Auto (PID simple) ----------------

    private void autoControlMotors(double desiredPositionRotations) {
        double clampedTargetRotations =
                MathUtil.clamp(
                        desiredPositionRotations,
                        CLIMBER_SOFT_LIMIT_MINIMUM_ROTATIONS,
                        CLIMBER_SOFT_LIMIT_MAXIMUM_ROTATIONS
                );

        lastRequestedClimberTargetRotations = clampedTargetRotations;
        climberHardwareInterface.setClimberPositionPidRotations(lastRequestedClimberTargetRotations);
    }

    public void autoExpand() {
        autoControlMotors(ClimberConstants.CLIMBER_EXTENDED_POSITION);
    }

    public void autoRetract() {
        autoControlMotors(CLIMBER_RETRACTED_TARGET_ROTATIONS);
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

    public double getLastRequestedTargetRotations() {
        return lastRequestedClimberTargetRotations;
    }
}