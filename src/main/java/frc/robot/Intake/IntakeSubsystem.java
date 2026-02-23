package frc.robot.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.AutoLogger.IntakeIOInputsAutoLogged;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Intake.IO.IntakeIO;

public final class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO intakeHardwareInterface;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean isRollerEnabledRequest = false;

    // normalized 0..1 rotations
    private double pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;

    private final PIDController pivotPositionController =
            new PIDController(
                    IntakeConstants.PIVOT_POSITION_PROPORTIONAL_GAIN,
                    IntakeConstants.PIVOT_POSITION_INTEGRAL_GAIN,
                    IntakeConstants.PIVOT_POSITION_DERIVATIVE_GAIN
            );

    private boolean isFullyDeployed = false;
    private boolean isFullyRetracted = true;

    private boolean didBootInitialization = false;

    public IntakeSubsystem(IntakeIO intakeHardwareInterface) {
        this.intakeHardwareInterface = intakeHardwareInterface;
        pivotPositionController.setIntegratorRange(-12.0, 12.0);
    }

    @Override
    public void periodic() {
        intakeHardwareInterface.updateInputs(inputs);

        if (!didBootInitialization) {
            performBootInitialization();
            didBootInitialization = true;
        }

        if (DriverStation.isDisabled()) {
            intakeHardwareInterface.stopRoller();
            intakeHardwareInterface.stopPivot();
            pivotPositionController.reset();
            intakeHardwareInterface.setRunning(false);
            Logger.processInputs("Intake", inputs);
            return;
        }

        updateDeploymentStateFromPosition();
        applyPivotPidControlOnly();
        applyRollerControl();
        applyMapleSimIntakeState();

        Logger.processInputs("Intake", inputs);
    }

    private void performBootInitialization() {
        if (IntakeConstants.PIVOT_ZERO_ENCODER_ON_BOOT_TO_RETRACT) {
            intakeHardwareInterface.setPivotRawEncoderPositionRotations(IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS);
        }
        pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;
        pivotPositionController.reset();
    }

    private void applyPivotPidControlOnly() {
        double measuredPositionRotations = inputs.pivotPositionRotations;

        double clampedTargetRotations =
                MathUtil.clamp(
                        pivotTargetPositionRotationsRequest,
                        IntakeConstants.PIVOT_SOFT_LIMIT_MINIMUM_ROTATIONS,
                        IntakeConstants.PIVOT_SOFT_LIMIT_MAXIMUM_ROTATIONS
                );

        double feedbackVolts =
                pivotPositionController.calculate(measuredPositionRotations, clampedTargetRotations);

        double totalCommandedVolts =
                MathUtil.clamp(
                        feedbackVolts,
                        -IntakeConstants.PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS,
                        IntakeConstants.PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS
                );

        intakeHardwareInterface.setPivotVoltage(totalCommandedVolts);

        inputs.pivotTargetPositionRotations = clampedTargetRotations;
        inputs.pivotPositionErrorRotations = clampedTargetRotations - measuredPositionRotations;
        inputs.pivotFeedbackVolts = feedbackVolts;
        inputs.pivotTotalCommandedVolts = totalCommandedVolts;
    }

    private void applyRollerControl() {
        if (!isRollerEnabledRequest) {
            intakeHardwareInterface.stopRoller();
            return;
        }
        intakeHardwareInterface.setRollerVoltage(IntakeConstants.INTAKE_ACTIVATION_VOLTAGE_VOLTS);
    }

    private void applyMapleSimIntakeState() {
        // Only run MapleSim intake when roller is enabled and intake is deployed
        boolean shouldRunIntakeSimulation = isRollerEnabledRequest && isFullyDeployed;
        intakeHardwareInterface.setRunning(shouldRunIntakeSimulation);
    }

    private void updateDeploymentStateFromPosition() {
        double positionRotations = inputs.pivotPositionRotations;

        double deployTargetRotations = IntakeConstants.PIVOT_DEPLOY_POSITION_ROTATIONS;
        double retractTargetRotations = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;

        double toleranceRotations = IntakeConstants.PIVOT_POSITION_TOLERANCE_ROTATIONS;
        double hysteresisRotations = IntakeConstants.PIVOT_POSITION_HYSTERESIS_ROTATIONS;

        boolean isAtDeploy = positionRotations >= (deployTargetRotations - toleranceRotations);
        boolean isAtRetract = positionRotations <= (retractTargetRotations + toleranceRotations);

        if (isAtDeploy) {
            isFullyDeployed = true;
            isFullyRetracted = false;
        } else if (positionRotations < (deployTargetRotations - toleranceRotations - hysteresisRotations)) {
            isFullyDeployed = false;
        }

        if (isAtRetract) {
            isFullyRetracted = true;
            isFullyDeployed = false;
        } else if (positionRotations > (retractTargetRotations + toleranceRotations + hysteresisRotations)) {
            isFullyRetracted = false;
        }
    }

    // ---------------- Requests ----------------

    public void requestDeployIntake() {
        pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_DEPLOY_POSITION_ROTATIONS;
    }

    public void requestRetractIntake() {
        pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;
    }

    public void setRollerEnabled(boolean enabled) {
        isRollerEnabledRequest = enabled;
    }

    // ---------------- State getters ----------------

    public boolean isIntakeDeployed() {
        return isFullyDeployed;
    }

    public boolean isIntakeRetracted() {
        return isFullyRetracted;
    }

    public boolean isFuelInsideIntake() {
        return inputs.isFuelInsideIntake;
    }

    public double getPivotTargetPositionRotations() {
        return pivotTargetPositionRotationsRequest;
    }
}