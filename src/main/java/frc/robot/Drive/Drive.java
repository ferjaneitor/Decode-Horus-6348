// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.AutoLogger.GyroIOInputsAutoLogged;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Mode;
import frc.robot.Drive.Generated.TunerConstants;
import frc.robot.Drive.Gyro.GyroIO;
import frc.robot.Drive.SwerveModule.ModuleIO;
import frc.robot.Util.LocalADStarAK;

// MapleSim

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

public class Drive extends SubsystemBase {
  public static final double ODOMETRY_FREQUENCY =
      TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KILOGRAMS = DriveConstants.ROBOT_MASS_KG;
  private static final double ROBOT_MOMENT_OF_INERTIA = DriveConstants.ROBOT_MOI;
  private static final double WHEEL_COEFFICIENT_OF_FRICTION = DriveConstants.WHEEL_COF;

  private static final RobotConfig pathPlannerRobotConfig =
      new RobotConfig(
          ROBOT_MASS_KILOGRAMS,
          ROBOT_MOMENT_OF_INERTIA,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COEFFICIENT_OF_FRICTION,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final frc.robot.Drive.SwerveModule.Module[] modules =
      new frc.robot.Drive.SwerveModule.Module[4]; // FL, FR, BL, BR

  private SysIdRoutine sysIdRoutine;

  private boolean hasOdometryThreadBeenStarted = false;
  private boolean hasPathPlannerBeenConfigured = false;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final Consumer<Pose2d> simulationWorldPoseConsumer;

  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeftModuleIo,
      ModuleIO frontRightModuleIo,
      ModuleIO backLeftModuleIo,
      ModuleIO backRightModuleIo) {

    this(gyroIO, frontLeftModuleIo, frontRightModuleIo, backLeftModuleIo, backRightModuleIo, (pose) -> {});
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeftModuleIo,
      ModuleIO frontRightModuleIo,
      ModuleIO backLeftModuleIo,
      ModuleIO backRightModuleIo,
      Consumer<Pose2d> simulationWorldPoseConsumer) {

    this.gyroIO = gyroIO;
    this.simulationWorldPoseConsumer = simulationWorldPoseConsumer;

    modules[0] = new frc.robot.Drive.SwerveModule.Module(frontLeftModuleIo, 0, TunerConstants.FrontLeft);
    modules[1] = new frc.robot.Drive.SwerveModule.Module(frontRightModuleIo, 1, TunerConstants.FrontRight);
    modules[2] = new frc.robot.Drive.SwerveModule.Module(backLeftModuleIo, 2, TunerConstants.BackLeft);
    modules[3] = new frc.robot.Drive.SwerveModule.Module(backRightModuleIo, 3, TunerConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
  }

  public void initializeIfNeeded() {
    startOdometryThreadIfNeeded();
    configurePathPlannerIfNeeded();
    configureSysIdRoutineIfNeeded();
  }

  private void startOdometryThreadIfNeeded() {
    if (hasOdometryThreadBeenStarted) {
      return;
    }
    PhoenixOdometryThread.getInstance().start();
    hasOdometryThreadBeenStarted = true;
  }

  private void configurePathPlannerIfNeeded() {
    if (hasPathPlannerBeenConfigured) {
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
              Constants.AutoConstants.TRANSLATION_KP,
              Constants.AutoConstants.TRANSLATION_KI,
              Constants.AutoConstants.TRANSLATION_KD
            ),
            new PIDConstants(
              Constants.AutoConstants.ROTATION_KP,
              Constants.AutoConstants.ROTATION_KI,
              Constants.AutoConstants.ROTATION_KD
            )),
        pathPlannerRobotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(Pose2d[]::new)));

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    hasPathPlannerBeenConfigured = true;
  }

  private void configureSysIdRoutineIfNeeded() {
    if (sysIdRoutine != null) {
      return;
    }

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null,
                this));
  }

  @Override
  public void periodic() {
    initializeIfNeeded();

    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[sampleIndex];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[sampleIndex];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[sampleIndex], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && DriveConstants.CURRENT_MODE != Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    configureSysIdRoutineIfNeeded();
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdRoutine.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    configureSysIdRoutineIfNeeded();
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdRoutine.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Odometry/YawRateRadPerSec")
  public double getYawRateRadiansPerSecond() {
    if (gyroInputs.connected) {
      return gyroInputs.yawVelocityRadPerSec;
    }
    ChassisSpeeds measuredChassisSpeeds = getChassisSpeeds();
    return measuredChassisSpeeds.omegaRadiansPerSecond;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredPublic")
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return getChassisSpeeds();
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds robotRelativeSpeeds = getRobotRelativeChassisSpeeds();
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond,
        getRotation());
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    simulationWorldPoseConsumer.accept(pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // MapleSim drivetrain configuration helper
  // MapleSim drivetrain configuration helper
  public static DriveTrainSimulationConfig getMapleSimConfig() {
    double trackLengthMeters =
        Math.abs(TunerConstants.FrontLeft.LocationX - TunerConstants.BackLeft.LocationX);

    double trackWidthMeters =
        Math.abs(TunerConstants.FrontLeft.LocationY - TunerConstants.FrontRight.LocationY);

    // Wheel properties (you can tune these)
    double wheelRadiusMeters = TunerConstants.FrontLeft.WheelRadius;

    // This is what you had as "0.860 kg" (wheel mass). MapleSim now wants wheel MOI instead.
    double wheelMassKilograms = 0.860;

    // Approximate wheel as a solid disk: I = 1/2 * m * r^2
    // If your wheel behaves more like a hoop, use I = m * r^2 instead.
    double wheelMomentOfInertiaKilogramSquareMeters =
        0.5 * wheelMassKilograms * wheelRadiusMeters * wheelRadiusMeters;

    // If you prefer, you can reuse your constant wheelCoefficientOfFriction,
    // but I’m keeping your original 0.01 (very slippery) as-is:
    double wheelCoefficientOfFrictionForSimulation = 0.01;

    SwerveModuleSimulationConfig swerveModuleSimulationConfig =
        new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX60Foc(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(0.2),
            Volts.of(0.2),
            Meters.of(wheelRadiusMeters),
            KilogramSquareMeters.of(wheelMomentOfInertiaKilogramSquareMeters),
            wheelCoefficientOfFrictionForSimulation);

    return DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withRobotMass(Kilograms.of(ROBOT_MASS_KILOGRAMS))
        .withTrackLengthTrackWidth(Meters.of(trackLengthMeters), Meters.of(trackWidthMeters))
        .withBumperSize(Meters.of(0.762), Meters.of(0.762))
        .withSwerveModule(swerveModuleSimulationConfig)
        .withCustomModuleTranslations(getModuleTranslations());
  }
}
