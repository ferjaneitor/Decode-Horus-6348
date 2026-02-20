// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.SuperSubsystem.SuperVision.VisionStandardDeviationModel;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Climber.ExpandClimberCmd;
import frc.robot.Climber.IO.ClimberIO;
import frc.robot.Climber.IO.ClimberIOSim;
import frc.robot.Climber.IO.ClimberIOSpark;
import frc.robot.Climber.RetractClimberCmd;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldCosntants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Drive.Drive;
import frc.robot.Drive.DriveCommands;
import frc.robot.Drive.Generated.TunerConstants;
import frc.robot.Drive.Gyro.GyroIO;
import frc.robot.Drive.Gyro.GyroIOPigeon2;
import frc.robot.Drive.Gyro.GyroIOSim;
import frc.robot.Drive.SwerveModule.ModuleIO;
import frc.robot.Drive.SwerveModule.ModuleIOTalonFX;
import frc.robot.Drive.SwerveModule.ModuleIOTalonFXSim;
import frc.robot.Intake.ActivateIntakeCmd;
import frc.robot.Intake.DeployIntakeCmd;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.RetractIntakeCmd;
import frc.robot.Intake.IO.IntakeIO;
import frc.robot.Intake.IO.IntakeIOSim;
import frc.robot.Intake.IO.IntakeIOSpark;
import frc.robot.Shooting.Hood.HoodCmd;
import frc.robot.Shooting.Hood.HoodSubsystem;
import frc.robot.Shooting.Hood.IO.HoodIO;
import frc.robot.Shooting.Hood.IO.HoodIOSim;
import frc.robot.Shooting.Hood.IO.HoodIOTalonFx;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Vision.VisionHardwareFactoryImpl;
import frc.robot.Vision.VisionSubsystem;

public class RobotContainer {
  private final Drive drive;

  // MapleSim drivetrain (SIM only)
  private final SwerveDriveSimulation swerveDriveSimulation;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final VisionStandardDeviationModel visionStandardDeviationModel =
      new VisionStandardDeviationModel(
          VisionConstants.MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG,
          VisionConstants.MAXIMUM_Z_ERROR_METERS,
          VisionConstants.MAXIMUM_OBSERVATION_AGE_SECONDS,
          VisionConstants.MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS,
          VisionConstants.MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS,
          VisionConstants.MAXIMUM_YAW_RATE_RADIANS_PER_SECOND,
          VisionConstants.MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS,
          VisionConstants.MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS
      );

  private final VisionSubsystem visionSubsystem;
  private final VisionHardwareFactoryImpl visionHardwareFactory;

  private final ShootingHelper shootingHelper =
      new ShootingHelper(FieldCosntants.IS_ANDYMARK_FIELD);

  private final HoodSubsystem hoodSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (DriveConstants.CURRENT_MODE) {
      case SIM -> {
        swerveDriveSimulation =
            new SwerveDriveSimulation(Drive.getMapleSimConfig(), Pose2d.kZero);

        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

        GyroIO gyroIoImplementation = new GyroIOSim(swerveDriveSimulation.getGyroSimulation());

        ModuleIO frontLeftModuleIo =
            new ModuleIOTalonFXSim(TunerConstants.FrontLeft, swerveDriveSimulation.getModules()[0]);
        ModuleIO frontRightModuleIo =
            new ModuleIOTalonFXSim(TunerConstants.FrontRight, swerveDriveSimulation.getModules()[1]);
        ModuleIO backLeftModuleIo =
            new ModuleIOTalonFXSim(TunerConstants.BackLeft, swerveDriveSimulation.getModules()[2]);
        ModuleIO backRightModuleIo =
            new ModuleIOTalonFXSim(TunerConstants.BackRight, swerveDriveSimulation.getModules()[3]);

        drive =
            new Drive(
                gyroIoImplementation,
                frontLeftModuleIo,
                frontRightModuleIo,
                backLeftModuleIo,
                backRightModuleIo,
                swerveDriveSimulation::setSimulationWorldPose);
      }
      case REAL -> {
        swerveDriveSimulation = null;

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
      }
      default -> {
        swerveDriveSimulation = null;

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    drive.initializeIfNeeded();

    VisionSubsystem.VisionPoseMeasurementConsumer visionPoseMeasurementConsumer =
        (visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations) ->
            drive.addVisionMeasurement(
                visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations);

    visionHardwareFactory =
        switch (DriveConstants.CURRENT_MODE) {
          case REAL -> new VisionHardwareFactoryImpl(drive::getPose, false);
          case SIM -> new VisionHardwareFactoryImpl(drive::getPose, true);
          default -> new VisionHardwareFactoryImpl(drive::getPose, false);
        };

    visionSubsystem =
        new VisionSubsystem(
            FieldCosntants.kTagLayout,
            FieldCosntants.FIELD_LENGTH_METERS,
            FieldCosntants.FIELD_WIDTH_METERS,
            () -> drive.getPose(),
            () -> drive.getYawRateRadiansPerSecond(),
            visionPoseMeasurementConsumer,
            visionStandardDeviationModel,
            VisionConstants.cameraSpecificationsList,
            visionHardwareFactory);

    HoodIO hoodIoImplementation =
        switch (DriveConstants.CURRENT_MODE) {
          case REAL -> new HoodIOTalonFx();
          case SIM -> new HoodIOSim();
          default -> new HoodIO() {};
        };

    hoodSubsystem = new HoodSubsystem(hoodIoImplementation);

    IntakeIO intakeIoImplementation =
        switch (DriveConstants.CURRENT_MODE) {
          case REAL ->
              new IntakeIOSpark(
                  new SuperSparkMax(
                      IntakeConstants.INTAKE_MOTOR_ID,
                      MotorType.kBrushless,
                      IntakeConstants.INTAKE_MOTOR_CONFIG()),
                  new SuperSparkMax(
                      IntakeConstants.PIVOT_INTAKE_MOTOR_ID,
                      MotorType.kBrushless,
                      IntakeConstants.PIVOT_INTAKE_MOTOR_CONFIG()));
          case SIM -> new IntakeIOSim(swerveDriveSimulation);
          default -> new IntakeIO() {};
        };

    intakeSubsystem = new IntakeSubsystem(intakeIoImplementation);

    ClimberIO climberIoImplementation =
        switch (DriveConstants.CURRENT_MODE) {
          case REAL ->
              new ClimberIOSpark(
                  new SuperSparkMax(
                      Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID,
                      MotorType.kBrushless,
                      Constants.ClimberConstants.CLIMBER_MOTOR_CONFIG()),
                  new SuperSparkMax(
                      Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR_ID,
                      MotorType.kBrushless,
                      Constants.ClimberConstants.CLIMBER_MOTOR_CONFIG()));
          case SIM -> new ClimberIOSim();
          default -> new ClimberIO() {};
        };

    climberSubsystem = new ClimberSubsystem(climberIoImplementation);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithVisionAim(
            drive,
            visionSubsystem,
            shootingHelper,
            () -> controller.x().getAsBoolean(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .povUp()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    controller.povRight().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.rightTrigger().whileTrue(new HoodCmd(hoodSubsystem, visionSubsystem, shootingHelper));

    controller.leftTrigger().whileTrue(new ActivateIntakeCmd(intakeSubsystem));
    controller.b().onTrue(new DeployIntakeCmd(intakeSubsystem));
    controller.a().onTrue(new RetractIntakeCmd(intakeSubsystem));
    
    controller.leftBumper().whileTrue(new ExpandClimberCmd(climberSubsystem));
    controller.rightBumper().whileTrue(new RetractClimberCmd(climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Call from Robot.simulationPeriodic()
  public void updateSimulation() {
    if (swerveDriveSimulation == null) {
      return;
    }

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPose", swerveDriveSimulation.getSimulatedDriveTrainPose());
  }

  // Call from Robot.disabledInit()
  public void resetSimulation() {
    if (swerveDriveSimulation == null) {
      return;
    }

    swerveDriveSimulation.setSimulationWorldPose(drive.getPose());
  }
}
