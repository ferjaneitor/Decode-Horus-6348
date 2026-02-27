// File: src/main/java/frc/robot/Shooting/Hood/IO/HoodIOSim.java
package frc.robot.Shooting.Hood.IO;

public class HoodIOSim implements HoodIO {

    private double simulatedLeftWheelVelocityRotationsPerSecond = 0.0;
    private double simulatedRightWheelVelocityRotationsPerSecond = 0.0;
    private double simulatedHoodAnglePositionRotations = 0.0;

    private double simulatedLeftWheelAppliedVolts = 0.0;
    private double simulatedRightWheelAppliedVolts = 0.0;
    private double simulatedHoodAngleAppliedVolts = 0.0;
    private double simulatedIndexerAppliedVolts = 0.0;

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {

        if (outputs.controlMode == HoodIOControlMode.DISABLED) {
            simulatedLeftWheelVelocityRotationsPerSecond = 0.0;
            simulatedRightWheelVelocityRotationsPerSecond = 0.0;

            simulatedLeftWheelAppliedVolts = 0.0;
            simulatedRightWheelAppliedVolts = 0.0;
            simulatedHoodAngleAppliedVolts = 0.0;
            simulatedIndexerAppliedVolts = 0.0;
            return;
        }

        // Instant response in simulation: no motor dynamics modeled.
        // Keep the same sign convention as real code.
        simulatedLeftWheelVelocityRotationsPerSecond = outputs.goalWheelRotationsPerSecond;
        simulatedRightWheelVelocityRotationsPerSecond = -outputs.goalWheelRotationsPerSecond;

        simulatedHoodAnglePositionRotations = outputs.goalHoodAnglePositionRotations;

        // Optional: fake voltages for log visibility.
        simulatedLeftWheelAppliedVolts = 6.0;
        simulatedRightWheelAppliedVolts = 6.0;
        simulatedHoodAngleAppliedVolts = 4.0;
        simulatedIndexerAppliedVolts = outputs.indexerEnabled ? 6.0 : 0.0;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {

        inputs.hoodAngleMotorConnected = true;
        inputs.leftWheelMotorConnected = true;
        inputs.rightWheelMotorConnected = true;
        inputs.indexerMotorConnected = true;

        inputs.hoodAnglePositionRotations = simulatedHoodAnglePositionRotations;
        inputs.hoodAngleVelocityRotationsPerSecond = 0.0;

        inputs.leftWheelVelocityRotationsPerSecond = simulatedLeftWheelVelocityRotationsPerSecond;
        inputs.rightWheelVelocityRotationsPerSecond = simulatedRightWheelVelocityRotationsPerSecond;

        inputs.hoodAngleAppliedVolts = simulatedHoodAngleAppliedVolts;
        inputs.leftWheelAppliedVolts = simulatedLeftWheelAppliedVolts;
        inputs.rightWheelAppliedVolts = simulatedRightWheelAppliedVolts;
        inputs.indexerAppliedVolts = simulatedIndexerAppliedVolts;

        inputs.hoodAngleSupplyCurrentAmps = 0.0;
        inputs.leftWheelSupplyCurrentAmps = 0.0;
        inputs.rightWheelSupplyCurrentAmps = 0.0;
        inputs.indexerSupplyCurrentAmps = 0.0;

        inputs.hoodAngleTorqueCurrentAmps = 0.0;
        inputs.leftWheelTorqueCurrentAmps = 0.0;
        inputs.rightWheelTorqueCurrentAmps = 0.0;
        inputs.indexerTorqueCurrentAmps = 0.0;

        inputs.hoodAngleTempCelsius = 0.0;
        inputs.leftWheelTempCelsius = 0.0;
        inputs.rightWheelTempCelsius = 0.0;
        inputs.indexerTempCelsius = 0.0;
    }
}