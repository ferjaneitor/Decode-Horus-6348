package frc.robot.Intake.IO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        // Roller
        public boolean rollerConnected = false;
        public double rollerVelocityRotationsPerSecond = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;

        // Pivot (normalized 0..1 rotations for your calibration workflow)
        public boolean pivotConnected = false;
        public double pivotPositionRotations = 0.0;
        public double pivotVelocityRotationsPerSecond = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        // Pivot raw (debug + future calibration)
        public double pivotRawEncoderPositionRotations = 0.0;
        public double pivotRawEncoderVelocityRotationsPerSecond = 0.0;

        // PID telemetry (filled by the subsystem)
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

    // Calibration support (REAL should implement; SIM can implement as offset)
    public default void setPivotRawEncoderPositionRotations(double rawEncoderPositionRotations) {}

    // MapleSim (SIM) only
    public default void setRunning(boolean runIntake) {}
    public default boolean isFuelInsideIntake() { return false; }
}