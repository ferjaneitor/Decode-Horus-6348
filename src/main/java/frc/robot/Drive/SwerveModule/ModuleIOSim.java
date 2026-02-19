// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive.SwerveModule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_PROPORTIONAL_GAIN = 0.05;
  private static final double DRIVE_DERIVATIVE_GAIN = 0.0;
  private static final double DRIVE_STATIC_GAIN = 0.0;
  private static final double DRIVE_VELOCITY_GAIN_ROTATIONS =
      0.91035; // (volt * secs) / rotation
  private static final double DRIVE_VELOCITY_GAIN_RADIANS =
      1.0 / Units.rotationsToRadians(1.0 / DRIVE_VELOCITY_GAIN_ROTATIONS);

  private static final double TURN_PROPORTIONAL_GAIN = 8.0;
  private static final double TURN_DERIVATIVE_GAIN = 0.0;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;

  private final PIDController driveController =
      new PIDController(DRIVE_PROPORTIONAL_GAIN, 0.0, DRIVE_DERIVATIVE_GAIN);

  private final PIDController turnController =
      new PIDController(TURN_PROPORTIONAL_GAIN, 0.0, TURN_DERIVATIVE_GAIN);

  private double driveFeedforwardVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
            DRIVE_GEARBOX);

    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            TURN_GEARBOX);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFeedforwardVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update sim state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Drive inputs
    inputs.driveConnected = true;

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.drivePositionRads = inputs.drivePositionRad;

    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveVelocityRadsPerSec = inputs.driveVelocityRadPerSec;

    inputs.driveAppliedVolts = driveAppliedVolts;

    double driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.driveCurrentAmps = driveCurrentAmps;
    inputs.driveSupplyCurrentAmps = driveCurrentAmps;
    inputs.driveTorqueCurrentAmps = 0.0;
    inputs.driveTempCelsius = 25.0;

    // Turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;

    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnAbsolutePositionRads = inputs.turnAbsolutePosition.getRadians();

    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPositionRads = inputs.turnPosition.getRadians();

    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnVelocityRadsPerSec = inputs.turnVelocityRadPerSec;

    inputs.turnAppliedVolts = turnAppliedVolts;

    double turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turnCurrentAmps = turnCurrentAmps;
    inputs.turnSupplyCurrentAmps = turnCurrentAmps;
    inputs.turnTorqueCurrentAmps = 0.0;
    inputs.turnTempCelsius = 25.0;

    // Odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFeedforwardVolts =
        DRIVE_STATIC_GAIN * Math.signum(velocityRadPerSec) + DRIVE_VELOCITY_GAIN_RADIANS * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
