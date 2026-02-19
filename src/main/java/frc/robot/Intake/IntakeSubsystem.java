package frc.robot.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.AutoLogger.IntakeIOInputsAutoLogged;
import frc.robot.Constants.IntakeConstants;

public final class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO intakeHardwareInterface;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Roller request
    private boolean isRollerEnabledRequest = false;

    // Pivot request
    private double pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROT;

    // Simple PID (RIO)
    private final PIDController pivotPositionController =
            new PIDController(
                    IntakeConstants.PIVOT_POSITION_PID_PROPORTIONAL_GAIN,
                    IntakeConstants.PIVOT_POSITION_PID_INTEGRAL_GAIN,
                    IntakeConstants.PIVOT_POSITION_PID_DERIVATIVE_GAIN
            );

    // State flags
    private boolean isFullyDeployed = false;
    private boolean isFullyRetracted = true;

    public IntakeSubsystem(IntakeIO intakeHardwareInterface) {
        this.intakeHardwareInterface = intakeHardwareInterface;

        // Opcional: evita windup si un día le metes integral
        pivotPositionController.setIntegratorRange(-12.0, 12.0);
    }

    @Override
    public void periodic() {
        // Read sensors / sim
        intakeHardwareInterface.updateInputs(inputs);

        // Update state flags
        updateDeploymentStateFromPosition();

        // Apply pivot PID ALWAYS (this is the whole point)
        applyPivotPidControl();

        // Apply roller
        applyRollerControl();

        // MapleSim intake on/off (SIM)
        applyMapleSimIntakeState();

        // Log
        Logger.processInputs("Intake", inputs);
    }

    private void applyPivotPidControl() {
        double measuredPositionRotations = inputs.pivotPositionRotations;

        double feedbackVolts =
                pivotPositionController.calculate(
                        measuredPositionRotations,
                        pivotTargetPositionRotationsRequest
                );

        double totalCommandedVolts = MathUtil.clamp(feedbackVolts, -12.0, 12.0);

        intakeHardwareInterface.setPivotVoltage(totalCommandedVolts);

        // Telemetry
        inputs.pivotTargetPositionRotations = pivotTargetPositionRotationsRequest;
        inputs.pivotPositionErrorRotations = pivotTargetPositionRotationsRequest - measuredPositionRotations;
        inputs.pivotFeedbackVolts = feedbackVolts;
        inputs.pivotTotalCommandedVolts = totalCommandedVolts;
    }

    private void applyRollerControl() {
        if (!isRollerEnabledRequest) {
            intakeHardwareInterface.stopRoller();
            return;
        }
        intakeHardwareInterface.setRollerVoltage(IntakeConstants.INTAKE_ACTIVATION_VOLTAGE);
    }

    private void applyMapleSimIntakeState() {
        // En REAL es no-op.
        boolean shouldRunIntakeSimulation = isRollerEnabledRequest && isFullyDeployed;
        intakeHardwareInterface.setRunning(shouldRunIntakeSimulation);
    }

    private void updateDeploymentStateFromPosition() {
        double positionRotations = inputs.pivotPositionRotations;

        double deployTargetRotations = IntakeConstants.PIVOT_DEPLOY_POSITION_ROT;
        double retractTargetRotations = IntakeConstants.PIVOT_RETRACT_POSITION_ROT;

        double toleranceRotations = IntakeConstants.PIVOT_POSITION_TOLERANCE_ROT;
        double hysteresisRotations = IntakeConstants.PIVOT_POSITION_HYSTERESIS_ROT;

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

    // ---------------- Requests (called by Commands) ----------------

    public void requestDeployIntake() {
        pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_DEPLOY_POSITION_ROT;
    }

    public void requestRetractIntake() {
        pivotTargetPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROT;
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
