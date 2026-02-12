package frc.AutoLogger;

import java.util.Arrays;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Drive.Gyro.GyroIO;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);

    table.put("YawPosition", yawPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("RollPosition", rollPosition);

    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    table.put("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    table.put("RollVelocityRadPerSec", rollVelocityRadPerSec);

    table.put("OdometryYawTimestamps", odometryYawTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);

    table.put("OdometryPitchTimestamps", odometryPitchTimestamps);
    table.put("OdometryPitchPositions", odometryPitchPositions);

    table.put("OdometryRollTimestamps", odometryRollTimestamps);
    table.put("OdometryRollPositions", odometryRollPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);

    yawPosition = table.get("YawPosition", yawPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    rollPosition = table.get("RollPosition", rollPosition);

    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);

    odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);

    odometryPitchTimestamps = table.get("OdometryPitchTimestamps", odometryPitchTimestamps);
    odometryPitchPositions = table.get("OdometryPitchPositions", odometryPitchPositions);

    odometryRollTimestamps = table.get("OdometryRollTimestamps", odometryRollTimestamps);
    odometryRollPositions = table.get("OdometryRollPositions", odometryRollPositions);
  }

  @Override
  public GyroIOInputsAutoLogged clone() throws CloneNotSupportedException {
    GyroIOInputsAutoLogged copy;
    try {
      copy = (GyroIOInputsAutoLogged) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone not supported", e);
    }

    copy.connected = this.connected;

    copy.yawPosition = this.yawPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.rollPosition = this.rollPosition;

    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    copy.pitchVelocityRadPerSec = this.pitchVelocityRadPerSec;
    copy.rollVelocityRadPerSec = this.rollVelocityRadPerSec;

    copy.odometryYawTimestamps = Arrays.copyOf(this.odometryYawTimestamps, this.odometryYawTimestamps.length);
    copy.odometryYawPositions = Arrays.copyOf(this.odometryYawPositions, this.odometryYawPositions.length);

    copy.odometryPitchTimestamps = Arrays.copyOf(this.odometryPitchTimestamps, this.odometryPitchTimestamps.length);
    copy.odometryPitchPositions = Arrays.copyOf(this.odometryPitchPositions, this.odometryPitchPositions.length);

    copy.odometryRollTimestamps = Arrays.copyOf(this.odometryRollTimestamps, this.odometryRollTimestamps.length);
    copy.odometryRollPositions = Arrays.copyOf(this.odometryRollPositions, this.odometryRollPositions.length);

    return copy;
  }
}
