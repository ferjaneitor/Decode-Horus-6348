package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.SuperSubsystem.SuperVision.VisionEntries;
import frc.SuperSubsystem.SuperVision.VisionEnums;

public class Constants {

    public static final class DriveConstants{
        public static final Mode SIM_MODE = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }

        public static final double ROBOT_MASS_KG = 74.088;
        public static final double ROBOT_MOI = 6.883;
        public static final double WHEEL_COF = 1.2;

        public static final double STEER_GAINS_KP = 100;
        public static final double STEER_GAINS_KI = 0.0;
        public static final double STEER_GAINS_KD = 0.5;
        public static final double STEER_GAINS_KS = 0.1;
        public static final double STEER_GAINS_KV = 1.91;
        public static final double STEER_GAINS_KA = 0.0;

        public static final double DRIVE_GAINS_KP = 0.1;
        public static final double DRIVE_GAINS_KI = 0.0;
        public static final double DRIVE_GAINS_KD = 0.0;
        public static final double DRIVE_GAINS_KS = 0.0;
        public static final double DRIVE_GAINS_KV = 0.124;
        public static final double DRIVE_GAINS_KA = 0.0;

        public static final Current kSlipCurrent = Amps.of(120.0);
        public static final Current SteeringStatorCurrentLimit = Amps.of(60.0);

        public static final String CAN_BUS = "CANivore";
        public static final String HOOT_FILE_PATH = "./logs/example.hoot";

        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

        public static final double K_COUPLE_RATIO = 3.8181818181818183;
        public static final double K_DRIVE_GEAR_RATIO = 7.363636363636365;
        public static final double K_STEER_GEAR_RATIO = 15.42857142857143;
        public static final Distance kWheelRadius = Inches.of(2.167);

        public static final boolean K_INVERT_LEFT_SIDE = false;
        public static final boolean K_INVERT_RIGHT_SIDE = true;

        public static final int K_PIGEON_ID = 1;

        // These are only used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final Distance kTrackWidth = Inches.of(20);
        public static final Distance kWheelBase = Inches.of(20);

        // Front Left
        public static final int K_FRONT_LEFT_DRIVE_MOTOR_ID = 3;
        public static final int K_FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int K_FRONT_LEFT_ENCODER_ID = 1;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
        public static final boolean K_FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_LEFT_ENCODER_INVERTED = false;

        public static final Distance kFrontLeftXPos = kWheelBase.div(2);
        public static final Distance kFrontLeftYPos = kTrackWidth.div(2);

        // Front Right
        public static final int K_FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
        public static final int K_FRONT_RIGHT_STEER_MOTOR_ID = 0;
        public static final int K_FRONT_RIGHT_ENCODER_ID = 0;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
        public static final boolean K_FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_RIGHT_ENCODER_INVERTED = false;

        public static final Distance kFrontRightXPos = kWheelBase.div(2);
        public static final Distance kFrontRightYPos = kTrackWidth.div(-2);

        // Back Left
        public static final int K_BACK_LEFT_DRIVE_MOTOR_ID = 7;
        public static final int K_BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int K_BACK_LEFT_ENCODER_ID = 3;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
        public static final boolean K_BACK_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_LEFT_ENCODER_INVERTED = false;

        public static final Distance kBackLeftXPos = kWheelBase.div(-2);
        public static final Distance kBackLeftYPos = kTrackWidth.div(2);

        // Back Right
        public static final int K_BACK_RIGHT_DRIVE_MOTOR_ID = 5;
        public static final int K_BACK_RIGHT_STEER_MOTOR_ID = 4;
        public static final int K_BACK_RIGHT_ENCODER_ID = 2;
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.17236328125);
        public static final boolean K_BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_RIGHT_ENCODER_INVERTED = false;

        public static final Distance kBackRightXPos = kWheelBase.div(-2);
        public static final Distance kBackRightYPos = kTrackWidth.div(-2);

        public static final double DEADBAND = 0.1;
        public static final double ANGLE_KP = 5.0;
        public static final double ANGLE_KD = 0.4;
        public static final double ANGLE_MAX_VELOCITY = 8.0;
        public static final double ANGLE_MAX_ACCELERATION = 20.0;
        public static final double FF_START_DELAY = 2.0; // Secs
        public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    }

    public static final class AutoConstants{
        public static final double TRANSLATION_KP = 5.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD  = 0.0;

        public static final double ROTATION_KP = 5.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
    }

    public static final class ShootingConstants {

        public static final double TARGET_POSITION_BLUE_ALLIANCE_WELDED_X_METERS = 11.915394;
        public static final double TARGET_POSITION_BLUE_ALLIANCE_WELDED_Y_METERS = 4.034663;

        public static final double TARGET_POSITION_RED_ALLIANCE_WELDED_X_METERS = 4.625594;
        public static final double TARGET_POSITION_RED_ALLIANCE_WELDED_Y_METERS = 4.034663;

        public static final double TARGET_POSITION_BLUE_ALLIANCE_ANDYMARK_X_METERS = 11.901424;
        public static final double TARGET_POSITION_BLUE_ALLIANCE_ANDYMARK_Y_METERS = 4.034663;

        public static final double TARGET_POSITION_RED_ALLIANCE_ANDYMARK_X_METERS = 4.611624;
        public static final double TARGET_POSITION_RED_ALLIANCE_ANDYMARK_Y_METERS = 4.034663;

        public static final double TARGET_HEIGHT_METERS = 1.8288;
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 22 * 0.0254;

        public static final double GRAVITY = 9.81;
        public static final double APEX_MAX_HEIGHT_METERS = 2.4;

        public static final double APEX_RELATIVE_TO_SHOOTER =
            APEX_MAX_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;

        public static final double APEX_TO_TARGET =
            APEX_MAX_HEIGHT_METERS - TARGET_HEIGHT_METERS;

        public static final double VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND =
            Math.sqrt(2 * GRAVITY * APEX_RELATIVE_TO_SHOOTER);

        public static final double TIME_GOING_UP =
            VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND / GRAVITY;   

        public static final double TIME_GOING_DOWN =
            Math.sqrt(2 * APEX_TO_TARGET / GRAVITY);

        public static final double TOTAL_TIME_SECONDS =
            TIME_GOING_UP + TIME_GOING_DOWN;

        public static final double MAX_RPM = 6000;
        public static final double WHEEL_DIAMETER_METERS = 4 * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double EXIT_SPEED_CONVERSION =
            WHEEL_CIRCUMFERENCE / 60.0;

        public static final double MAXIMUM_EXIT_SPEED_METERS_PER_SECOND =
            MAX_RPM * EXIT_SPEED_CONVERSION * 0.60;

        public static final double MINIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(0);
        public static final double MAXIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(90);

        public static final double MINIMUM_HORIZONTAL_DISTANCE_METERS = 0.1;
    }

        public static final class VisionConstants {

        @SuppressWarnings("null")
        public static final List<VisionEntries.CameraSpecifications> cameraSpecificationsList =
                List.of(
                        new VisionEntries.CameraSpecifications(
                                "FrontTagCam",
                                /* cameraToRobotTransform3d */ new Transform3d(),
                                VisionEnums.PoseEstimateNoiseLevel.MEDIUM,
                                /* cameraConfidenceMultiplier */ 1.0
                        )
                        // Agrega más cámaras aquí...
                );

        public static final double MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG = 0.20;
        public static final double MAXIMUM_Z_ERROR_METERS = 0.25;
        public static final double MAXIMUM_OBSERVATION_AGE_SECONDS = 0.35;

        public static final double MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS = 4.0;
        public static final double MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS = 6.0;
        public static final double MAXIMUM_YAW_RATE_RADIANS_PER_SECOND = 4.0;

        public static final double MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS = 2.0;
        public static final double MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS = 3.0;
    }

    public static final class FieldCosntants{
        
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;

        public static final boolean IS_ANDYMARK_FIELD = true ;

        public static long[] getShootingValidTagIdentifiers() {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == Alliance.Red
                ? new long[] {2, 5, 8, 9, 10, 11}
                : new long[] {18, 21, 24, 25, 26, 27};
        }

    }

}
