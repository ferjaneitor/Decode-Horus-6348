package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static final class DriveConstants{
        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
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

}
