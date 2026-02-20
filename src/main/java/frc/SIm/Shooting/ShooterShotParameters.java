package frc.SIm.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record ShooterShotParameters(
    double exitSpeedMetersPerSecond,
    double hoodAngleRadiansFromBallisticModel,
    double launchHeightMeters,
    Pose2d robotPoseField,
    ChassisSpeeds robotFieldRelativeSpeeds
) {}