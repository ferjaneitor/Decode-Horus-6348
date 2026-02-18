package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.AutoLogger.IntakeIOInputsAutoLogged;
import frc.robot.Constants.IntakeConstants;

public final class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO intakeHardwareInterface;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean isFullyDeployed = false;
    private boolean isFullyRetracted = true;

    private double lastRequestedPivotTargetRotations = 0.0;

    public IntakeSubsystem(IntakeIO intakeHardwareInterface) {
        this.intakeHardwareInterface = intakeHardwareInterface;
        this.lastRequestedPivotTargetRotations = IntakeConstants.PIVOT_RETRACT_POSITION_ROT;
    }

    @Override
    public void periodic() {
        intakeHardwareInterface.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        updateDeploymentStateFromPosition();
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

    // ---------------- Roller ----------------

    public void activateIntake() {
        intakeHardwareInterface.setRollerVoltage(IntakeConstants.INTAKE_ACTIVATION_VOLTAGE);
    }

    public void stopIntake() {
        intakeHardwareInterface.stopRoller();
    }

    // ---------------- Pivot (human-proof) ----------------

    public void deployIntake() {
    lastRequestedPivotTargetRotations = IntakeConstants.PIVOT_DEPLOY_POSITION_ROT;
        intakeHardwareInterface.setPivotPositionPidRotations(lastRequestedPivotTargetRotations);
    }

    public void retractIntake() {
        lastRequestedPivotTargetRotations = IntakeConstants.PIVOT_RETRACT_POSITION_ROT;
        intakeHardwareInterface.setPivotPositionPidRotations(lastRequestedPivotTargetRotations);
    }

    public void stopPivot() {
        intakeHardwareInterface.stopPivot();
    }

    // ---------------- State ----------------

    public double getIntakePositionRotations() {
        return inputs.pivotPositionRotations;
    }

    public boolean isIntakeDeployed() {
        return isFullyDeployed;
    }

    public boolean isIntakeRetracted() {
        return isFullyRetracted;
    }
}
