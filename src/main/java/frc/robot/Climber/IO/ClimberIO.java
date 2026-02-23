package frc.robot.Climber.IO;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    public enum ClimberControlMode {
        OPEN_LOOP_VOLTAGE,
        POSITION_PID
    }

    @AutoLog
    public static class ClimberIOInputs {
        // Left motor telemetry
        public boolean leftClimberConnected = false;
        public double leftClimberPositionRotations = 0.0;
        public double leftClimberVelocityRotationsPerSecond = 0.0;
        public double leftClimberAppliedVolts = 0.0;
        public double leftClimberCurrentAmps = 0.0;

        // Right motor telemetry
        public boolean rightClimberConnected = false;
        public double rightClimberPositionRotations = 0.0;
        public double rightClimberVelocityRotationsPerSecond = 0.0;
        public double rightClimberAppliedVolts = 0.0;
        public double rightClimberCurrentAmps = 0.0;

        // Control state (for logging)
        public ClimberControlMode climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;
        public double climberTargetPositionRotations = 0.0;
        public double climberAppliedVoltageCommandVolts = 0.0;

        // PID telemetry
        public double leftFeedbackVolts = 0.0;
        public double leftTotalCommandedVolts = 0.0;

        public double rightFeedbackVolts = 0.0;
        public double rightTotalCommandedVolts = 0.0;
    }

    public default void updateInputs(ClimberIOInputs climberInputs) {}

    public default void setClimberVoltage(double voltage) {}
    public default void stopClimber() {}

    public default void setClimberPositionPidRotations(double targetPositionRotations) {}

    // Calibration helper (REAL should implement; SIM can implement as offset)
    public default void setClimberRawEncoderPositionRotations(double leftPositionRotations, double rightPositionRotations) {}
}