package frc.AutoLogger;

import java.util.Arrays;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Drive.SwerveModule.ModuleIO;
import frc.robot.Drive.SwerveModule.ModuleIO.ModuleIOInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveConnected", driveConnected);
    table.put("DrivePositionRads", drivePositionRads);
    table.put("DriveVelocityRadsPerSec", driveVelocityRadsPerSec);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    table.put("DriveTorqueCurrentAmps", driveTorqueCurrentAmps);
    table.put("DriveTempCelsius", driveTempCelsius);

    table.put("TurnConnected", turnConnected);
    table.put("TurnEncoderConnected", turnEncoderConnected);
    table.put("TurnAbsolutePositionRads", turnAbsolutePositionRads);
    table.put("TurnPositionRads", turnPositionRads);
    table.put("TurnVelocityRadsPerSec", turnVelocityRadsPerSec);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    table.put("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    table.put("TurnTempCelsius", turnTempCelsius);

    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    table.put("OdometryTurnPositions", odometryTurnPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    driveConnected = table.get("DriveConnected", driveConnected);
    drivePositionRads = table.get("DrivePositionRads", drivePositionRads);
    driveVelocityRadsPerSec = table.get("DriveVelocityRadsPerSec", driveVelocityRadsPerSec);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    driveTorqueCurrentAmps = table.get("DriveTorqueCurrentAmps", driveTorqueCurrentAmps);
    driveTempCelsius = table.get("DriveTempCelsius", driveTempCelsius);

    turnConnected = table.get("TurnConnected", turnConnected);
    turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
    turnAbsolutePositionRads = table.get("TurnAbsolutePositionRads", turnAbsolutePositionRads);
    turnPositionRads = table.get("TurnPositionRads", turnPositionRads);
    turnVelocityRadsPerSec = table.get("TurnVelocityRadsPerSec", turnVelocityRadsPerSec);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    turnTorqueCurrentAmps = table.get("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    turnTempCelsius = table.get("TurnTempCelsius", turnTempCelsius);

    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
  }

  @Override
  public ModuleIOInputsAutoLogged clone() throws CloneNotSupportedException {
    ModuleIOInputsAutoLogged copy = (ModuleIOInputsAutoLogged) super.clone();

    copy.driveConnected = this.driveConnected;
    copy.drivePositionRads = this.drivePositionRads;
    copy.driveVelocityRadsPerSec = this.driveVelocityRadsPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveSupplyCurrentAmps = this.driveSupplyCurrentAmps;
    copy.driveTorqueCurrentAmps = this.driveTorqueCurrentAmps;
    copy.driveTempCelsius = this.driveTempCelsius;

    copy.turnConnected = this.turnConnected;
    copy.turnEncoderConnected = this.turnEncoderConnected;
    copy.turnAbsolutePositionRads = this.turnAbsolutePositionRads;
    copy.turnPositionRads = this.turnPositionRads;
    copy.turnVelocityRadsPerSec = this.turnVelocityRadsPerSec;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnSupplyCurrentAmps = this.turnSupplyCurrentAmps;
    copy.turnTorqueCurrentAmps = this.turnTorqueCurrentAmps;
    copy.turnTempCelsius = this.turnTempCelsius;

    copy.odometryTimestamps = Arrays.copyOf(this.odometryTimestamps, this.odometryTimestamps.length);
    copy.odometryDrivePositionsRad = Arrays.copyOf(this.odometryDrivePositionsRad, this.odometryDrivePositionsRad.length);
    copy.odometryTurnPositions = Arrays.copyOf(this.odometryTurnPositions, this.odometryTurnPositions.length);

    return copy;
  }
}
