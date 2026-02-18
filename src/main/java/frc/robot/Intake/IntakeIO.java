package frc.robot.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    public enum PivotControlMode {
        OPEN_LOOP_VOLTAGE,
        POSITION_PID
    }

    @AutoLog
    public static class IntakeIOInputs {
        public boolean rollerConnected = false;
        public double rollerVelocityRotationsPerSecond = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;

        public boolean pivotConnected = false;
        public double pivotPositionRotations = 0.0;
        public double pivotVelocityRotationsPerSecond = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public PivotControlMode pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
        public double pivotTargetPositionRotations = 0.0;

        // Telemetry PID simple
        public double pivotPositionErrorRotations = 0.0;
        public double feedbackVolts = 0.0;
        public double totalCommandedVolts = 0.0;
    }

    public default void updateInputs(IntakeIOInputs intakeInputs) {}

    public default void setRollerVoltage(double voltage) {}
    public default void stopRoller() {}

    public default void setPivotVoltage(double voltage) {}
    public default void stopPivot() {}

    public default void setPivotPositionPidRotations(double targetPositionRotations) {}
}
