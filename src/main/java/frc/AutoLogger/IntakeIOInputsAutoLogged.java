package frc.AutoLogger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Intake.IntakeIO;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("RollerConnected", rollerConnected);
        table.put("RollerVelocityRotationsPerSecond", rollerVelocityRotationsPerSecond);
        table.put("RollerAppliedVolts", rollerAppliedVolts);
        table.put("RollerCurrentAmps", rollerCurrentAmps);

        table.put("PivotConnected", pivotConnected);
        table.put("PivotPositionRotations", pivotPositionRotations);
        table.put("PivotVelocityRotationsPerSecond", pivotVelocityRotationsPerSecond);
        table.put("PivotAppliedVolts", pivotAppliedVolts);
        table.put("PivotCurrentAmps", pivotCurrentAmps);

        table.put("PivotControlModeOrdinal", pivotControlMode != null ? pivotControlMode.ordinal() : -1);
        table.put("PivotTargetPositionRotations", pivotTargetPositionRotations);

        table.put("PivotPositionErrorRotations", pivotPositionErrorRotations);
        table.put("FeedbackVolts", feedbackVolts);
        table.put("TotalCommandedVolts", totalCommandedVolts);
    }

    @Override
    public void fromLog(LogTable table) {
        rollerConnected = table.get("RollerConnected", rollerConnected);
        rollerVelocityRotationsPerSecond =
                table.get("RollerVelocityRotationsPerSecond", rollerVelocityRotationsPerSecond);
        rollerAppliedVolts = table.get("RollerAppliedVolts", rollerAppliedVolts);
        rollerCurrentAmps = table.get("RollerCurrentAmps", rollerCurrentAmps);

        pivotConnected = table.get("PivotConnected", pivotConnected);
        pivotPositionRotations = table.get("PivotPositionRotations", pivotPositionRotations);
        pivotVelocityRotationsPerSecond =
                table.get("PivotVelocityRotationsPerSecond", pivotVelocityRotationsPerSecond);
        pivotAppliedVolts = table.get("PivotAppliedVolts", pivotAppliedVolts);
        pivotCurrentAmps = table.get("PivotCurrentAmps", pivotCurrentAmps);

        int pivotControlModeOrdinal = table.get("PivotControlModeOrdinal", -1);
        if (pivotControlModeOrdinal >= 0
                && pivotControlModeOrdinal < IntakeIO.PivotControlMode.values().length) {
            pivotControlMode = IntakeIO.PivotControlMode.values()[pivotControlModeOrdinal];
        }

        pivotTargetPositionRotations =
                table.get("PivotTargetPositionRotations", pivotTargetPositionRotations);

        pivotPositionErrorRotations =
                table.get("PivotPositionErrorRotations", pivotPositionErrorRotations);
        feedbackVolts = table.get("FeedbackVolts", feedbackVolts);
        totalCommandedVolts = table.get("TotalCommandedVolts", totalCommandedVolts);
    }

    @Override
    public IntakeIOInputsAutoLogged clone() throws CloneNotSupportedException {
        IntakeIOInputsAutoLogged copy = (IntakeIOInputsAutoLogged) super.clone();

        copy.rollerConnected = this.rollerConnected;
        copy.rollerVelocityRotationsPerSecond = this.rollerVelocityRotationsPerSecond;
        copy.rollerAppliedVolts = this.rollerAppliedVolts;
        copy.rollerCurrentAmps = this.rollerCurrentAmps;

        copy.pivotConnected = this.pivotConnected;
        copy.pivotPositionRotations = this.pivotPositionRotations;
        copy.pivotVelocityRotationsPerSecond = this.pivotVelocityRotationsPerSecond;
        copy.pivotAppliedVolts = this.pivotAppliedVolts;
        copy.pivotCurrentAmps = this.pivotCurrentAmps;

        copy.pivotControlMode = this.pivotControlMode;
        copy.pivotTargetPositionRotations = this.pivotTargetPositionRotations;

        copy.pivotPositionErrorRotations = this.pivotPositionErrorRotations;
        copy.feedbackVolts = this.feedbackVolts;
        copy.totalCommandedVolts = this.totalCommandedVolts;

        return copy;
    }
}
