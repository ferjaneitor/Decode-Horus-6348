// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Vision.VisionSubsystem;

public class DriveCommands {

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY,
                DriveConstants.ANGLE_MAX_ACCELERATION));

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        Commands.run(() -> drive.runCharacterization(0.0), drive)
            .withTimeout(DriveConstants.FF_START_DELAY),

        Commands.runOnce(timer::restart),

        Commands.run(
                () -> {
                  double voltage = timer.get() * DriveConstants.FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)
            .finallyDo(
                () -> {
                  int sampleCount = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;

                  for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
                    sumX += velocitySamples.get(sampleIndex);
                    sumY += voltageSamples.get(sampleIndex);
                    sumXY += velocitySamples.get(sampleIndex) * voltageSamples.get(sampleIndex);
                    sumX2 += velocitySamples.get(sampleIndex) * velocitySamples.get(sampleIndex);
                  }

                  double kS = (sumY * sumX2 - sumX * sumXY) / (sampleCount * sumX2 - sumX * sumX);
                  double kV = (sampleCount * sumXY - sumX * sumY) / (sampleCount * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> limiter.reset(0.0)),
            Commands.run(
                () -> {
                  double speed = limiter.calculate(DriveConstants.WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        Commands.sequence(
            Commands.waitSeconds(1.0),

            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int index = 0; index < 4; index++) {
                        wheelDelta += Math.abs(positions[index] - state.positions[index]) / 4.0;
                      }

                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println("********** Wheel Radius Characterization Results **********");
                      System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  public static Command joystickDriveWithVisionAim(
      Drive drive,
      VisionSubsystem visionSubsystem,
      ShootingHelper shootingHelper,
      BooleanSupplier isAutoAimActiveSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier) {

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(),
                drive.getMaxAngularSpeedRadPerSec() * 2.0));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    final boolean[] autoAimWasActiveLastLoop = new boolean[] {false};

    return Commands.run(
            () -> {
              Pose2d robotPose = drive.getPose();
              ChassisSpeeds fieldSpeeds = drive.getFieldRelativeChassisSpeeds();

              shootingHelper.update(robotPose, fieldSpeeds);

              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              boolean isAutoAimButtonPressed = isAutoAimActiveSupplier.getAsBoolean();
              boolean isPossibleToShoot = shootingHelper.isPossibleToShoot();
              boolean isAValidShootingTarget = visionSubsystem.itsAValidShootingTarget();

              boolean shouldAutoAim = isAutoAimButtonPressed && isPossibleToShoot && isAValidShootingTarget;

              double thetaRadiansPerSecond;

              if (shouldAutoAim) {
                double desiredHeadingRadians = shootingHelper.getDesiredChassisHeadingRadians();
                double currentHeadingRadians = drive.getRotation().getRadians();

                if (!autoAimWasActiveLastLoop[0]) {
                  thetaController.reset(currentHeadingRadians);
                }

                thetaRadiansPerSecond =
                    thetaController.calculate(currentHeadingRadians, desiredHeadingRadians);
              } else {
                double manualThetaNormalized =
                    MathUtil.applyDeadband(thetaSupplier.getAsDouble(), DriveConstants.DEADBAND);

                manualThetaNormalized =
                    Math.copySign(manualThetaNormalized * manualThetaNormalized, manualThetaNormalized);

                thetaRadiansPerSecond = manualThetaNormalized * drive.getMaxAngularSpeedRadPerSec();

                if (autoAimWasActiveLastLoop[0]) {
                  thetaController.reset(drive.getRotation().getRadians());
                }
              }

              autoAimWasActiveLastLoop[0] = shouldAutoAim;

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      thetaRadiansPerSecond);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(() -> thetaController.reset(drive.getRotation().getRadians()));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               