// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.SuperSubsystem.SuperVision.VisionStandardDeviationModel;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldCosntants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Drive.Drive;
import frc.robot.Drive.DriveCommands;
import frc.robot.Drive.Generated.TunerConstants;
import frc.robot.Drive.Gyro.GyroIO;
import frc.robot.Drive.Gyro.GyroIOPigeon2;
import frc.robot.Drive.SwerveModule.ModuleIO;
import frc.robot.Drive.SwerveModule.ModuleIOSim;
import frc.robot.Drive.SwerveModule.ModuleIOTalonFX;
import frc.robot.Intake.IntakeIO;
import frc.robot.Intake.IntakeIOSim;
import frc.robot.Intake.IntakeIOSpark;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Shooting.Hood.HoodCmd;
import frc.robot.Shooting.Hood.HoodIO;
import frc.robot.Shooting.Hood.HoodIOSim;
import frc.robot.Shooting.Hood.HoodIOTalonFx;
import frc.robot.Shooting.Hood.HoodSubsystem;
import frc.robot.Vision.VisionHardwareFactoryImpl;
import frc.robot.Vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;

    // Controller
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
        ); // Placeholder values, tune based on your vision system's performance

    private final VisionSubsystem visionSubsystem;

    private final VisionHardwareFactoryImpl visionHardwareFactory;

    private final ShootingHelper shootingHelper = new ShootingHelper(FieldCosntants.IS_ANDYMARK_FIELD); // Set to true if using Andymark target

    private final HoodSubsystem hoodSubsystem ;

    private final IntakeSubsystem intakeSubsystem;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive = switch (DriveConstants.CURRENT_MODE) {
            case REAL -> new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            case SIM -> new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));
            default -> new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
        }; // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        // Sim robot, instantiate physics sim IO implementations
        // Replayed robot, disable IO implementations

    drive.initializeIfNeeded();

    

    VisionSubsystem.VisionPoseMeasurementConsumer visionPoseMeasurementConsumer =
            (visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations) -> {
              // Ajusta este método al nombre real de tu Drive
              drive.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStandardDeviations);
            };

    visionHardwareFactory =
        switch (DriveConstants.CURRENT_MODE) {
        case REAL -> new VisionHardwareFactoryImpl(drive::getPose, false);
        case SIM -> new VisionHardwareFactoryImpl(drive::getPose, true);
        default -> new VisionHardwareFactoryImpl(drive::getPose, false);
        };

    visionSubsystem = new VisionSubsystem(
        FieldCosntants.kTagLayout, 
        FieldCosntants.FIELD_LENGTH_METERS,
        FieldCosntants.FIELD_WIDTH_METERS, 
        () -> drive.getPose(), 
        () -> drive.getYawRateRadiansPerSecond(), 
        visionPoseMeasurementConsumer, 
        visionStandardDeviationModel, 
        VisionConstants.cameraSpecificationsList, 
        visionHardwareFactory);
    
    HoodIO hoodIo = switch (DriveConstants.CURRENT_MODE) {
        case REAL -> new HoodIOTalonFx();
        case SIM -> new HoodIOSim();
        default -> new HoodIO() {};
    };

    hoodSubsystem = new HoodSubsystem(hoodIo);

    IntakeIO intakeIo = switch (DriveConstants.CURRENT_MODE) {
        case REAL -> new IntakeIOSpark(
            new SuperSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, IntakeConstants.INTAKE_MOTOR_CONFIG()), 
            new SuperSparkMax(IntakeConstants.PIVOT_INTAKE_MOTOR_ID, MotorType.kBrushless, IntakeConstants.PIVOT_INTAKE_MOTOR_CONFIG())
        );
        case SIM -> new IntakeIOSim();
        default -> new IntakeIO() {};
    };

    intakeSubsystem = new IntakeSubsystem(intakeIo);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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

    // Configure the button bindings
    configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX())
    //     );

    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithVisionAim(
            drive, 
            visionSubsystem, 
            shootingHelper,
            () -> controller.rightBumper().getAsBoolean(), // Auto-aim while right bumper is held
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()
        )
    );

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    
    controller.rightTrigger().whileTrue(new HoodCmd(hoodSubsystem, visionSubsystem, shootingHelper));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    return autoChooser.get();
    }
}