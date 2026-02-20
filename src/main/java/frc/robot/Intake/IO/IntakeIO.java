package frc.robot.Intake.IO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

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

        // Telemetry simple del PID (llenado por el Subsystem)
        public double pivotTargetPositionRotations = 0.0;
        public double pivotPositionErrorRotations = 0.0;
        public double pivotFeedbackVolts = 0.0;
        public double pivotTotalCommandedVolts = 0.0;

        // MapleSim (SIM)
        public boolean isFuelInsideIntake = false;
        public int fuelCountInsideIntake = 0;
    }

    public default void updateInputs(IntakeIOInputs intakeInputs) {}

    public default void setRollerVoltage(double voltage) {}
    public default void stopRoller() {}

    public default void setPivotVoltage(double voltage) {}
    public default void stopPivot() {}

    // MapleSim (SIM) only
    public default void setRunning(boolean runIntake) {}
    public default boolean isFuelInsideIntake() { return false; }
}
