// File: src/main/java/frc/AutoLogger/ClimberIOInputsAutoLogged.java
package frc.AutoLogger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Climber.ClimberIO;

public final class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("LeftClimberConnected", leftClimberConnected);
        table.put("LeftClimberPositionRotations", leftClimberPositionRotations);
        table.put("LeftClimberVelocityRotationsPerSecond", leftClimberVelocityRotationsPerSecond);
        table.put("LeftClimberAppliedVolts", leftClimberAppliedVolts);
        table.put("LeftClimberCurrentAmps", leftClimberCurrentAmps);

        table.put("RightClimberConnected", rightClimberConnected);
        table.put("RightClimberPositionRotations", rightClimberPositionRotations);
        table.put("RightClimberVelocityRotationsPerSecond", rightClimberVelocityRotationsPerSecond);
        table.put("RightClimberAppliedVolts", rightClimberAppliedVolts);
        table.put("RightClimberCurrentAmps", rightClimberCurrentAmps);

        table.put("ClimberControlModeOrdinal", climberControlMode != null ? climberControlMode.ordinal() : -1);
        table.put("ClimberTargetPositionRotations", climberTargetPositionRotations);
        table.put("ClimberAppliedVoltageCommandVolts", climberAppliedVoltageCommandVolts);

        table.put("LeftFeedbackVolts", leftFeedbackVolts);
        table.put("LeftTotalCommandedVolts", leftTotalCommandedVolts);

        table.put("RightFeedbackVolts", rightFeedbackVolts);
        table.put("RightTotalCommandedVolts", rightTotalCommandedVolts);
    }

    @Override
    public void fromLog(LogTable table) {
        leftClimberConnected = table.get("LeftClimberConnected", leftClimberConnected);
        leftClimberPositionRotations = table.get("LeftClimberPositionRotations", leftClimberPositionRotations);
        leftClimberVelocityRotationsPerSecond =
                table.get("LeftClimberVelocityRotationsPerSecond", leftClimberVelocityRotationsPerSecond);
        leftClimberAppliedVolts = table.get("LeftClimberAppliedVolts", leftClimberAppliedVolts);
        leftClimberCurrentAmps = table.get("LeftClimberCurrentAmps", leftClimberCurrentAmps);

        rightClimberConnected = table.get("RightClimberConnected", rightClimberConnected);
        rightClimberPositionRotations = table.get("RightClimberPositionRotations", rightClimberPositionRotations);
        rightClimberVelocityRotationsPerSecond =
                table.get("RightClimberVelocityRotationsPerSecond", rightClimberVelocityRotationsPerSecond);
        rightClimberAppliedVolts = table.get("RightClimberAppliedVolts", rightClimberAppliedVolts);
        rightClimberCurrentAmps = table.get("RightClimberCurrentAmps", rightClimberCurrentAmps);

        int climberControlModeOrdinal = table.get("ClimberControlModeOrdinal", -1);
        if (climberControlModeOrdinal >= 0
                && climberControlModeOrdinal < ClimberIO.ClimberControlMode.values().length) {
            climberControlMode = ClimberIO.ClimberControlMode.values()[climberControlModeOrdinal];
        }

        climberTargetPositionRotations =
                table.get("ClimberTargetPositionRotations", climberTargetPositionRotations);

        climberAppliedVoltageCommandVolts =
                table.get("ClimberAppliedVoltageCommandVolts", climberAppliedVoltageCommandVolts);

        leftFeedbackVolts = table.get("LeftFeedbackVolts", leftFeedbackVolts);
        leftTotalCommandedVolts = table.get("LeftTotalCommandedVolts", leftTotalCommandedVolts);

        rightFeedbackVolts = table.get("RightFeedbackVolts", rightFeedbackVolts);
        rightTotalCommandedVolts = table.get("RightTotalCommandedVolts", rightTotalCommandedVolts);
    }

    @Override
    public ClimberIOInputsAutoLogged clone() throws CloneNotSupportedException {
        ClimberIOInputsAutoLogged copy = (ClimberIOInputsAutoLogged) super.clone();

        copy.leftClimberConnected = this.leftClimberConnected;
        copy.leftClimberPositionRotations = this.leftClimberPositionRotations;
        copy.leftClimberVelocityRotationsPerSecond = this.leftClimberVelocityRotationsPerSecond;
        copy.leftClimberAppliedVolts = this.leftClimberAppliedVolts;
        copy.leftClimberCurrentAmps = this.leftClimberCurrentAmps;

        copy.rightClimberConnected = this.rightClimberConnected;
        copy.rightClimberPositionRotations = this.rightClimberPositionRotations;
        copy.rightClimberVelocityRotationsPerSecond = this.rightClimberVelocityRotationsPerSecond;
        copy.rightClimberAppliedVolts = this.rightClimberAppliedVolts;
        copy.rightClimberCurrentAmps = this.rightClimberCurrentAmps;

        copy.climberControlMode = this.climberControlMode;
        copy.climberTargetPositionRotations = this.climberTargetPositionRotations;
        copy.climberAppliedVoltageCommandVolts = this.climberAppliedVoltageCommandVolts;

        copy.leftFeedbackVolts = this.leftFeedbackVolts;
        copy.leftTotalCommandedVolts = this.leftTotalCommandedVolts;

        copy.rightFeedbackVolts = this.rightFeedbackVolts;
        copy.rightTotalCommandedVolts = this.rightTotalCommandedVolts;

        return copy;
    }
}
