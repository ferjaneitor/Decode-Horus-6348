package frc.AutoLogger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Intake.IntakeIO;

public final class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {

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

        // Simple PID telemetry (now filled by the Subsystem)
        table.put("PivotTargetPositionRotations", pivotTargetPositionRotations);
        table.put("PivotPositionErrorRotations", pivotPositionErrorRotations);
        table.put("PivotFeedbackVolts", pivotFeedbackVolts);
        table.put("PivotTotalCommandedVolts", pivotTotalCommandedVolts);

        // MapleSim (SIM)
        table.put("IsFuelInsideIntake", isFuelInsideIntake);
        table.put("FuelCountInsideIntake", fuelCountInsideIntake);
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

        pivotTargetPositionRotations =
                table.get("PivotTargetPositionRotations", pivotTargetPositionRotations);
        pivotPositionErrorRotations =
                table.get("PivotPositionErrorRotations", pivotPositionErrorRotations);
        pivotFeedbackVolts = table.get("PivotFeedbackVolts", pivotFeedbackVolts);
        pivotTotalCommandedVolts = table.get("PivotTotalCommandedVolts", pivotTotalCommandedVolts);

        isFuelInsideIntake = table.get("IsFuelInsideIntake", isFuelInsideIntake);
        fuelCountInsideIntake = table.get("FuelCountInsideIntake", fuelCountInsideIntake);
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

        copy.pivotTargetPositionRotations = this.pivotTargetPositionRotations;
        copy.pivotPositionErrorRotations = this.pivotPositionErrorRotations;
        copy.pivotFeedbackVolts = this.pivotFeedbackVolts;
        copy.pivotTotalCommandedVolts = this.pivotTotalCommandedVolts;

        copy.isFuelInsideIntake = this.isFuelInsideIntake;
        copy.fuelCountInsideIntake = this.fuelCountInsideIntake;

        return copy;
    }
}
