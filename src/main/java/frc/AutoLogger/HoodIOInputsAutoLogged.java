package frc.AutoLogger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Shooting.Hood.IO.HoodIO.HoodIOInputs;

public class HoodIOInputsAutoLogged extends HoodIOInputs implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("HoodAngleMotorConnected", hoodAngleMotorConnected);
        table.put("LeftWheelMotorConnected", leftWheelMotorConnected);
        table.put("RightWheelMotorConnected", rightWheelMotorConnected);
        table.put("IndexerMotorConnected", indexerMotorConnected);

        table.put("HoodAnglePositionRotations", hoodAnglePositionRotations);
        table.put("HoodAngleVelocityRotationsPerSecond", hoodAngleVelocityRotationsPerSecond);

        table.put("LeftWheelVelocityRotationsPerSecond", leftWheelVelocityRotationsPerSecond);
        table.put("RightWheelVelocityRotationsPerSecond", rightWheelVelocityRotationsPerSecond);

        table.put("HoodAngleAppliedVolts", hoodAngleAppliedVolts);
        table.put("LeftWheelAppliedVolts", leftWheelAppliedVolts);
        table.put("RightWheelAppliedVolts", rightWheelAppliedVolts);
        table.put("IndexerAppliedVolts", indexerAppliedVolts);

        table.put("HoodAngleSupplyCurrentAmps", hoodAngleSupplyCurrentAmps);
        table.put("LeftWheelSupplyCurrentAmps", leftWheelSupplyCurrentAmps);
        table.put("RightWheelSupplyCurrentAmps", rightWheelSupplyCurrentAmps);
        table.put("IndexerSupplyCurrentAmps", indexerSupplyCurrentAmps);

        table.put("HoodAngleTorqueCurrentAmps", hoodAngleTorqueCurrentAmps);
        table.put("LeftWheelTorqueCurrentAmps", leftWheelTorqueCurrentAmps);
        table.put("RightWheelTorqueCurrentAmps", rightWheelTorqueCurrentAmps);
        table.put("IndexerTorqueCurrentAmps", indexerTorqueCurrentAmps);

        table.put("HoodAngleTempCelsius", hoodAngleTempCelsius);
        table.put("LeftWheelTempCelsius", leftWheelTempCelsius);
        table.put("RightWheelTempCelsius", rightWheelTempCelsius);
        table.put("IndexerTempCelsius", indexerTempCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
        hoodAngleMotorConnected = table.get("HoodAngleMotorConnected", hoodAngleMotorConnected);
        leftWheelMotorConnected = table.get("LeftWheelMotorConnected", leftWheelMotorConnected);
        rightWheelMotorConnected = table.get("RightWheelMotorConnected", rightWheelMotorConnected);
        indexerMotorConnected = table.get("IndexerMotorConnected", indexerMotorConnected);

        hoodAnglePositionRotations = table.get("HoodAnglePositionRotations", hoodAnglePositionRotations);
        hoodAngleVelocityRotationsPerSecond = table.get("HoodAngleVelocityRotationsPerSecond", hoodAngleVelocityRotationsPerSecond);

        leftWheelVelocityRotationsPerSecond = table.get("LeftWheelVelocityRotationsPerSecond", leftWheelVelocityRotationsPerSecond);
        rightWheelVelocityRotationsPerSecond = table.get("RightWheelVelocityRotationsPerSecond", rightWheelVelocityRotationsPerSecond);

        hoodAngleAppliedVolts = table.get("HoodAngleAppliedVolts", hoodAngleAppliedVolts);
        leftWheelAppliedVolts = table.get("LeftWheelAppliedVolts", leftWheelAppliedVolts);
        rightWheelAppliedVolts = table.get("RightWheelAppliedVolts", rightWheelAppliedVolts);
        indexerAppliedVolts = table.get("IndexerAppliedVolts", indexerAppliedVolts);

        hoodAngleSupplyCurrentAmps = table.get("HoodAngleSupplyCurrentAmps", hoodAngleSupplyCurrentAmps);
        leftWheelSupplyCurrentAmps = table.get("LeftWheelSupplyCurrentAmps", leftWheelSupplyCurrentAmps);
        rightWheelSupplyCurrentAmps = table.get("RightWheelSupplyCurrentAmps", rightWheelSupplyCurrentAmps);
        indexerSupplyCurrentAmps = table.get("IndexerSupplyCurrentAmps", indexerSupplyCurrentAmps);

        hoodAngleTorqueCurrentAmps = table.get("HoodAngleTorqueCurrentAmps", hoodAngleTorqueCurrentAmps);
        leftWheelTorqueCurrentAmps = table.get("LeftWheelTorqueCurrentAmps", leftWheelTorqueCurrentAmps);
        rightWheelTorqueCurrentAmps = table.get("RightWheelTorqueCurrentAmps", rightWheelTorqueCurrentAmps);
        indexerTorqueCurrentAmps = table.get("IndexerTorqueCurrentAmps", indexerTorqueCurrentAmps);

        hoodAngleTempCelsius = table.get("HoodAngleTempCelsius", hoodAngleTempCelsius);
        leftWheelTempCelsius = table.get("LeftWheelTempCelsius", leftWheelTempCelsius);
        rightWheelTempCelsius = table.get("RightWheelTempCelsius", rightWheelTempCelsius);
        indexerTempCelsius = table.get("IndexerTempCelsius", indexerTempCelsius);
    }

    @Override
    public HoodIOInputsAutoLogged clone() throws CloneNotSupportedException {
        HoodIOInputsAutoLogged copy = (HoodIOInputsAutoLogged) super.clone();

        copy.hoodAngleMotorConnected = this.hoodAngleMotorConnected;
        copy.leftWheelMotorConnected = this.leftWheelMotorConnected;
        copy.rightWheelMotorConnected = this.rightWheelMotorConnected;
        copy.indexerMotorConnected = this.indexerMotorConnected;

        copy.hoodAnglePositionRotations = this.hoodAnglePositionRotations;
        copy.hoodAngleVelocityRotationsPerSecond = this.hoodAngleVelocityRotationsPerSecond;

        copy.leftWheelVelocityRotationsPerSecond = this.leftWheelVelocityRotationsPerSecond;
        copy.rightWheelVelocityRotationsPerSecond = this.rightWheelVelocityRotationsPerSecond;

        copy.hoodAngleAppliedVolts = this.hoodAngleAppliedVolts;
        copy.leftWheelAppliedVolts = this.leftWheelAppliedVolts;
        copy.rightWheelAppliedVolts = this.rightWheelAppliedVolts;
        copy.indexerAppliedVolts = this.indexerAppliedVolts;

        copy.hoodAngleSupplyCurrentAmps = this.hoodAngleSupplyCurrentAmps;
        copy.leftWheelSupplyCurrentAmps = this.leftWheelSupplyCurrentAmps;
        copy.rightWheelSupplyCurrentAmps = this.rightWheelSupplyCurrentAmps;
        copy.indexerSupplyCurrentAmps = this.indexerSupplyCurrentAmps;

        copy.hoodAngleTorqueCurrentAmps = this.hoodAngleTorqueCurrentAmps;
        copy.leftWheelTorqueCurrentAmps = this.leftWheelTorqueCurrentAmps;
        copy.rightWheelTorqueCurrentAmps = this.rightWheelTorqueCurrentAmps;
        copy.indexerTorqueCurrentAmps = this.indexerTorqueCurrentAmps;

        copy.hoodAngleTempCelsius = this.hoodAngleTempCelsius;
        copy.leftWheelTempCelsius = this.leftWheelTempCelsius;
        copy.rightWheelTempCelsius = this.rightWheelTempCelsius;
        copy.indexerTempCelsius = this.indexerTempCelsius;

        return copy;
    }
}
