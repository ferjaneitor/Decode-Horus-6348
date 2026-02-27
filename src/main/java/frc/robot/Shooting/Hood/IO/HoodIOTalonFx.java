package frc.robot.Shooting.Hood.IO;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.HoodConstants;
import frc.robot.Drive.Generated.TunerConstants;

public class HoodIOTalonFx implements HoodIO {

    private final TalonFX hoodAngleMotorTalonFx;
    private final TalonFX rightWheelMotorTalonFx;
    private final TalonFX leftWheelMotorTalonFx;
    private final TalonFX indexerMotorTalonFx;

    private final VelocityTorqueCurrentFOC rightWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC leftWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final MotionMagicExpoTorqueCurrentFOC hoodAnglePositionRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final DutyCycleOut indexerDutyCycleRequest = new DutyCycleOut(0.0);

    public HoodIOTalonFx() {
        hoodAngleMotorTalonFx = new TalonFX(HoodConstants.HOOD_ANGLE_TALON_ID, TunerConstants.kCANBus);
        rightWheelMotorTalonFx = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, TunerConstants.kCANBus);
        leftWheelMotorTalonFx = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID, TunerConstants.kCANBus);
        indexerMotorTalonFx = new TalonFX(HoodConstants.INDEXER_TALON_ID, TunerConstants.kCANBus);

        TalonFXConfigurator hoodAngleConfigurator = hoodAngleMotorTalonFx.getConfigurator();
        TalonFXConfigurator rightWheelConfigurator = rightWheelMotorTalonFx.getConfigurator();
        TalonFXConfigurator leftWheelConfigurator = leftWheelMotorTalonFx.getConfigurator();
        TalonFXConfigurator indexerConfigurator = indexerMotorTalonFx.getConfigurator();

        hoodAngleConfigurator.apply(HoodConstants.HOOD_ANGLE_SLOT_CONFIGS);
        rightWheelConfigurator.apply(HoodConstants.RIGHT_HOOD_PROPULSION_SLOT_CONFIGS);
        leftWheelConfigurator.apply(HoodConstants.LEFT_HOOD_PROPULSION_SLOT_CONFIGS);
        indexerConfigurator.apply(HoodConstants.INDEXER_SLOT_CONFIGS);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodAngleMotorConnected = true;
        inputs.leftWheelMotorConnected = true;
        inputs.rightWheelMotorConnected = true;
        inputs.indexerMotorConnected = true;

        inputs.hoodAnglePositionRotations = hoodAngleMotorTalonFx.getPosition().getValueAsDouble();
        inputs.hoodAngleVelocityRotationsPerSecond = hoodAngleMotorTalonFx.getVelocity().getValueAsDouble();

        inputs.leftWheelVelocityRotationsPerSecond = leftWheelMotorTalonFx.getVelocity().getValueAsDouble();
        inputs.rightWheelVelocityRotationsPerSecond = rightWheelMotorTalonFx.getVelocity().getValueAsDouble();

        inputs.hoodAngleAppliedVolts = hoodAngleMotorTalonFx.getMotorVoltage().getValueAsDouble();
        inputs.leftWheelAppliedVolts = leftWheelMotorTalonFx.getMotorVoltage().getValueAsDouble();
        inputs.rightWheelAppliedVolts = rightWheelMotorTalonFx.getMotorVoltage().getValueAsDouble();
        inputs.indexerAppliedVolts = indexerMotorTalonFx.getMotorVoltage().getValueAsDouble();

        inputs.hoodAngleSupplyCurrentAmps = hoodAngleMotorTalonFx.getSupplyCurrent().getValueAsDouble();
        inputs.leftWheelSupplyCurrentAmps = leftWheelMotorTalonFx.getSupplyCurrent().getValueAsDouble();
        inputs.rightWheelSupplyCurrentAmps = rightWheelMotorTalonFx.getSupplyCurrent().getValueAsDouble();
        inputs.indexerSupplyCurrentAmps = indexerMotorTalonFx.getSupplyCurrent().getValueAsDouble();

        inputs.hoodAngleTorqueCurrentAmps = hoodAngleMotorTalonFx.getTorqueCurrent().getValueAsDouble();
        inputs.leftWheelTorqueCurrentAmps = leftWheelMotorTalonFx.getTorqueCurrent().getValueAsDouble();
        inputs.rightWheelTorqueCurrentAmps = rightWheelMotorTalonFx.getTorqueCurrent().getValueAsDouble();
        inputs.indexerTorqueCurrentAmps = indexerMotorTalonFx.getTorqueCurrent().getValueAsDouble();

        inputs.hoodAngleTempCelsius = hoodAngleMotorTalonFx.getDeviceTemp().getValueAsDouble();
        inputs.leftWheelTempCelsius = leftWheelMotorTalonFx.getDeviceTemp().getValueAsDouble();
        inputs.rightWheelTempCelsius = rightWheelMotorTalonFx.getDeviceTemp().getValueAsDouble();
        inputs.indexerTempCelsius = indexerMotorTalonFx.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        if (outputs.controlMode == HoodIOControlMode.DISABLED) {
            rightWheelMotorTalonFx.setControl(rightWheelVelocityRequest.withVelocity(0.0));
            leftWheelMotorTalonFx.setControl(leftWheelVelocityRequest.withVelocity(0.0));
            indexerMotorTalonFx.setControl(indexerDutyCycleRequest.withOutput(0.0));

            double currentHoodAnglePositionRotations = hoodAngleMotorTalonFx.getPosition().getValueAsDouble();
            hoodAngleMotorTalonFx.setControl(
                hoodAnglePositionRequest.withPosition(currentHoodAnglePositionRotations)
            );
            return;
        }

        rightWheelMotorTalonFx.setControl(
            rightWheelVelocityRequest.withVelocity(-outputs.goalWheelRotationsPerSecond)
        );
        leftWheelMotorTalonFx.setControl(
            leftWheelVelocityRequest.withVelocity(outputs.goalWheelRotationsPerSecond)
        );

        hoodAngleMotorTalonFx.setControl(
            hoodAnglePositionRequest.withPosition(outputs.goalHoodAnglePositionRotations)
        );

        double requestedDutyCycle = outputs.indexerEnabled ? HoodConstants.INDEXER_SPEED : 0.0;
        indexerMotorTalonFx.setControl(indexerDutyCycleRequest.withOutput(requestedDutyCycle));
    }
}