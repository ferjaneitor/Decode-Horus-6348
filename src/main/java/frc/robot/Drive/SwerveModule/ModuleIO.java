// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    // Drive
    public boolean driveConnected = false;

    // Keep original fields (singular)
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;

    // Add aliases expected by ModuleIOInputsAutoLogged (plural names)
    public double drivePositionRads = 0.0;
    public double driveVelocityRadsPerSec = 0.0;

    public double driveAppliedVolts = 0.0;

    // Keep original combined current field
    public double driveCurrentAmps = 0.0;

    // Add the more detailed fields expected by ModuleIOInputsAutoLogged
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    // Turn
    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;

    // Keep originals
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;

    // Add aliases expected by ModuleIOInputsAutoLogged (Rad doubles)
    public double turnAbsolutePositionRads = 0.0;
    public double turnPositionRads = 0.0;

    public double turnVelocityRadPerSec = 0.0;
    public double turnVelocityRadsPerSec = 0.0;

    public double turnAppliedVolts = 0.0;

    // Keep original combined current field
    public double turnCurrentAmps = 0.0;

    // Add more detailed fields expected by ModuleIOInputsAutoLogged
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;

    // Odometry
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
