// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive.Gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    // Position (Rotation2d)
    public Rotation2d yawPosition = Rotation2d.kZero;
    public Rotation2d pitchPosition = Rotation2d.kZero;
    public Rotation2d rollPosition = Rotation2d.kZero;

    // Angular velocity (rad/s)
    public double yawVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;

    // Odometry sampling (typically used for drivetrain pose estimation)
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    // Optional: if later you want time-synced pitch/roll too, you already have the slots
    public double[] odometryPitchTimestamps = new double[] {};
    public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};
    public double[] odometryRollTimestamps = new double[] {};
    public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
  