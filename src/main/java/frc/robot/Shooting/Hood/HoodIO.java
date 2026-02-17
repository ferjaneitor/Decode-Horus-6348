package frc.robot.Shooting.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        public boolean hoodAngleMotorConnected = false;
        public boolean leftWheelMotorConnected = false;
        public boolean rightWheelMotorConnected = false;
        public boolean indexerMotorConnected = false;

        // Phoenix default units (rotor units): rotations, rotations/sec
        public double hoodAnglePositionRotations = 0.0;
        public double hoodAngleVelocityRotationsPerSecond = 0.0;

        public double leftWheelVelocityRotationsPerSecond = 0.0;
        public double rightWheelVelocityRotationsPerSecond = 0.0;

        public double hoodAngleAppliedVolts = 0.0;
        public double leftWheelAppliedVolts = 0.0;
        public double rightWheelAppliedVolts = 0.0;
        public double indexerAppliedVolts = 0.0;

        public double hoodAngleSupplyCurrentAmps = 0.0;
        public double leftWheelSupplyCurrentAmps = 0.0;
        public double rightWheelSupplyCurrentAmps = 0.0;
        public double indexerSupplyCurrentAmps = 0.0;

        public double hoodAngleTorqueCurrentAmps = 0.0;
        public double leftWheelTorqueCurrentAmps = 0.0;
        public double rightWheelTorqueCurrentAmps = 0.0;
        public double indexerTorqueCurrentAmps = 0.0;

        public double hoodAngleTempCelsius = 0.0;
        public double leftWheelTempCelsius = 0.0;
        public double rightWheelTempCelsius = 0.0;
        public double indexerTempCelsius = 0.0;
    }

    public static enum HoodIOControlMode {
        DISABLED,
        RUNNING
    }

    public static class HoodIOOutputs {
        public HoodIOControlMode controlMode = HoodIOControlMode.DISABLED;

        // Setpoints in Phoenix default units (rotor units)
        public double goalWheelRotationsPerSecond = 0.0;
        public double goalHoodAnglePositionRotations = 0.0;

        public boolean indexerEnabled = false;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void applyOutputs(HoodIOOutputs outputs) {}
}
