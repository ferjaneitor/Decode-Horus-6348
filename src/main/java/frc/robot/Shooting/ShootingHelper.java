package frc.robot.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShootingConstants;

public final class ShootingHelper {

    private double hoodAngleRadians;
    private double horizontalVelocityMetersPerSecond;
    private final double verticalVelocityMetersPerSecond;
    private double exitSpeedMetersPerSecond;
    private double desiredChassisHeadingRadians;
    private double distanceToDigitalTargetMeters;

    private double digitalTargetXFieldMeters;
    private double digitalTargetYFieldMeters;

    private boolean isPossibleToShoot;

    private final boolean isAndymarkTarget;
    private final boolean isRedAlliance;

    public ShootingHelper(boolean isAndymarkTarget, boolean isRedAlliance) {
        this.isAndymarkTarget = isAndymarkTarget;
        this.isRedAlliance = isRedAlliance;

        this.verticalVelocityMetersPerSecond =
            ShootingConstants.VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND;
    }

    public void update(Pose2d robotPoseField, ChassisSpeeds fieldSpeeds) {

        double shooterXFieldMeters = robotPoseField.getX();
        double shooterYFieldMeters = robotPoseField.getY();

        double targetXFieldMeters = getTargetX();
        double targetYFieldMeters = getTargetY();

        // DIGITAL TARGET (constant time model)
        digitalTargetXFieldMeters =
            targetXFieldMeters - fieldSpeeds.vxMetersPerSecond * ShootingConstants.TOTAL_TIME_SECONDS;

        digitalTargetYFieldMeters =
            targetYFieldMeters - fieldSpeeds.vyMetersPerSecond * ShootingConstants.TOTAL_TIME_SECONDS;

        double deltaX = digitalTargetXFieldMeters - shooterXFieldMeters;
        double deltaY = digitalTargetYFieldMeters - shooterYFieldMeters;

        distanceToDigitalTargetMeters = Math.hypot(deltaX, deltaY);

        if (distanceToDigitalTargetMeters < ShootingConstants.MINIMUM_HORIZONTAL_DISTANCE_METERS) {
            isPossibleToShoot = false;
            return;
        }

        horizontalVelocityMetersPerSecond =
            distanceToDigitalTargetMeters / ShootingConstants.TOTAL_TIME_SECONDS;

        hoodAngleRadians = Math.atan2(
            verticalVelocityMetersPerSecond,
            horizontalVelocityMetersPerSecond
        );

        exitSpeedMetersPerSecond = Math.hypot(
            horizontalVelocityMetersPerSecond,
            verticalVelocityMetersPerSecond
        );

        desiredChassisHeadingRadians = Math.atan2(deltaY, deltaX);

        isPossibleToShoot =
            exitSpeedMetersPerSecond <= ShootingConstants.MAXIMUM_EXIT_SPEED_METERS_PER_SECOND &&
            hoodAngleRadians >= ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS &&
            hoodAngleRadians <= ShootingConstants.MAXIMUM_HOOD_ANGLE_RADIANS;
    }

    private double getTargetX() {
        if (isAndymarkTarget) {
            return isRedAlliance
                ? ShootingConstants.TARGET_POSITION_RED_ALLIANCE_ANDYMARK_X_METERS
                : ShootingConstants.TARGET_POSITION_BLUE_ALLIANCE_ANDYMARK_X_METERS;
        }
        return isRedAlliance
            ? ShootingConstants.TARGET_POSITION_RED_ALLIANCE_WELDED_X_METERS
            : ShootingConstants.TARGET_POSITION_BLUE_ALLIANCE_WELDED_X_METERS;
    }

    private double getTargetY() {
        if (isAndymarkTarget) {
            return isRedAlliance
                ? ShootingConstants.TARGET_POSITION_RED_ALLIANCE_ANDYMARK_Y_METERS
                : ShootingConstants.TARGET_POSITION_BLUE_ALLIANCE_ANDYMARK_Y_METERS;
        }
        return isRedAlliance
            ? ShootingConstants.TARGET_POSITION_RED_ALLIANCE_WELDED_Y_METERS
            : ShootingConstants.TARGET_POSITION_BLUE_ALLIANCE_WELDED_Y_METERS;
    }

    public double getHoodAngleRadians() {
        return hoodAngleRadians;
    }

    public double getExitSpeedMetersPerSecond() {
        return exitSpeedMetersPerSecond;
    }

    public double getDesiredChassisHeadingRadians() {
        return desiredChassisHeadingRadians;
    }

    public boolean isPossibleToShoot() {
        return isPossibleToShoot;
    }

    public double getDistanceToDigitalTargetMeters() {
        return distanceToDigitalTargetMeters;
    }
}
