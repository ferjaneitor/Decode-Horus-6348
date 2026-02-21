package frc.robot.Shooting.Hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.AutoLogger.HoodIOInputsAutoLogged;
import frc.SIm.Shooting.ShooterProjectileSimulation;
import frc.SIm.Shooting.ShooterShotParameters;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Shooting.Hood.IO.HoodIO;
import frc.robot.Vision.VisionSubsystem;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private enum HoodOperationalState {
        STOWED,
        AIMING_AND_SPINNING,
        POST_SHOT_COASTING,
        STOWING
    }

    private final HoodIO hoodIo;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO.HoodIOOutputs outputs = new HoodIO.HoodIOOutputs();

    private final Debouncer readyDebouncerSeconds =
        new Debouncer(0.12, Debouncer.DebounceType.kRising);

    private HoodOperationalState hoodOperationalState = HoodOperationalState.STOWED;

    private boolean isShootingRequestActive = false;

    // These are ALWAYS the selected setpoints (either live or default).
    private double desiredExitSpeedMetersPerSecond = ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;
    private double desiredHoodAngleRadiansFromModel = ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;

    private double postShotCoastStartTimestampSeconds = 0.0;
    private double hoodHoldPositionRotations = 0.0;

    @AutoLogOutput(key = "Hood/ReadyToShoot")
    private boolean readyToShoot = false;

    private ShooterProjectileSimulation shooterProjectileSimulation = (shotParameters) -> {};

    private Supplier<Pose2d> robotPoseFieldSupplier = () -> Pose2d.kZero;
    private Supplier<ChassisSpeeds> robotFieldRelativeChassisSpeedsSupplier = () -> new ChassisSpeeds();

    private double nextAllowedShotTimestampSeconds = 0.0;
    private boolean lastIndexerEnabledCommanded = false;

    private static final double SHOT_COOLDOWN_SECONDS = 0.25;

    public HoodSubsystem(HoodIO hoodIo) {
        this.hoodIo = hoodIo;
    }

    @Override
    public void periodic() {
        hoodIo.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (DriverStation.isDisabled()) {
            outputs.controlMode = HoodIO.HoodIOControlMode.DISABLED;
            outputs.goalWheelRotationsPerSecond = 0.0;
            outputs.goalHoodAnglePositionRotations = 0.0;
            outputs.indexerEnabled = false;

            readyToShoot = false;

            Logger.recordOutput("Hood/ControlMode", outputs.controlMode.toString());
            hoodIo.applyOutputs(outputs);
            return;
        }

        outputs.controlMode = HoodIO.HoodIOControlMode.RUNNING;

        // State transitions based on single-button request.
        if (isShootingRequestActive) {
            if (hoodOperationalState != HoodOperationalState.AIMING_AND_SPINNING) {
                hoodOperationalState = HoodOperationalState.AIMING_AND_SPINNING;
                readyDebouncerSeconds.calculate(false);
            }
        } else {
            if (hoodOperationalState == HoodOperationalState.AIMING_AND_SPINNING) {
                hoodOperationalState = HoodOperationalState.POST_SHOT_COASTING;
                postShotCoastStartTimestampSeconds = Timer.getFPGATimestamp();
                hoodHoldPositionRotations = inputs.hoodAnglePositionRotations;
                readyToShoot = false;
            }
        }

        switch (hoodOperationalState) {

            case AIMING_AND_SPINNING -> {
                // Wheels: m/s -> rotations/sec
                outputs.goalWheelRotationsPerSecond =
                    desiredExitSpeedMetersPerSecond * HoodConstants.CONVERSION_RATIO_FROM_METERS_TO_RPS;

                // Hood: model angle -> complementary -> subtract offset -> rotations
                double complementaryAngleRadians =
                    HoodConstants.COMPLEMENTARY_ANGLE - desiredHoodAngleRadiansFromModel;

                double correctedAngleRadians =
                    complementaryAngleRadians - HoodConstants.HOOD_ANGLE_OFFSET_RADIANS;

                outputs.goalHoodAnglePositionRotations =
                    correctedAngleRadians * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

                boolean rawReady =
                    isWheelVelocityReady(outputs.goalWheelRotationsPerSecond) &&
                    isHoodAngleReady(outputs.goalHoodAnglePositionRotations);

                readyToShoot = readyDebouncerSeconds.calculate(rawReady);

                // Indexer ONLY if the button is held AND we're ready.
                outputs.indexerEnabled = isShootingRequestActive && readyToShoot;

                maybeSimulateShot(outputs.indexerEnabled);
            }

            case POST_SHOT_COASTING -> {
                // 1) Cut indexer and wheels immediately
                outputs.indexerEnabled = false;
                outputs.goalWheelRotationsPerSecond = 0.0;

                // 2) Hold hood where it is
                outputs.goalHoodAnglePositionRotations = hoodHoldPositionRotations;

                // 3) Then go home after wheels slow enough (or timeout)
                double currentTimestampSeconds = Timer.getFPGATimestamp();
                double elapsedSeconds = currentTimestampSeconds - postShotCoastStartTimestampSeconds;

                boolean hasMinimumTimeElapsed =
                    elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MINIMUM_TIME_SECONDS;

                boolean hasMaximumTimeElapsed =
                    elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MAXIMUM_TIME_SECONDS;

                boolean wheelsAreSlowEnough = areWheelsBelowStopThreshold();

                if ((hasMinimumTimeElapsed && wheelsAreSlowEnough) || hasMaximumTimeElapsed) {
                    hoodOperationalState = HoodOperationalState.STOWING;
                }

                readyToShoot = false;
            }

            case STOWING -> {
                outputs.indexerEnabled = false;
                outputs.goalWheelRotationsPerSecond = 0.0;
                outputs.goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;

                readyToShoot = false;

                if (isHoodAngleReady(HoodConstants.HOOD_HOME_POSITION_ROTATIONS)) {
                    hoodOperationalState = HoodOperationalState.STOWED;
                }
            }

            case STOWED -> {
                outputs.indexerEnabled = false;
                outputs.goalWheelRotationsPerSecond = 0.0;
                outputs.goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;

                readyToShoot = false;
            }
        }

        Logger.recordOutput("Hood/OperationalState", hoodOperationalState.toString());
        Logger.recordOutput("Hood/ShootingRequestActive", isShootingRequestActive);

        Logger.recordOutput("Hood/DesiredExitSpeedMetersPerSecond", desiredExitSpeedMetersPerSecond);
        Logger.recordOutput("Hood/DesiredHoodAngleRadiansFromModel", desiredHoodAngleRadiansFromModel);

        Logger.recordOutput("Hood/GoalWheelRotationsPerSecond", outputs.goalWheelRotationsPerSecond);
        Logger.recordOutput("Hood/GoalHoodAnglePositionRotations", outputs.goalHoodAnglePositionRotations);
        Logger.recordOutput("Hood/IndexerEnabled", outputs.indexerEnabled);
        Logger.recordOutput("Hood/ReadyToShoot", readyToShoot);

        hoodIo.applyOutputs(outputs);
    }

    // Single-button interface
    public void setShootingRequestActive(boolean isActive) {
        isShootingRequestActive = isActive;

        if (isActive) {
            hoodOperationalState = HoodOperationalState.AIMING_AND_SPINNING;
        }
    }

    // CORE RULE:
    // If ANY live condition is false -> DEFAULT immediately.
    // No validation on default, because default is your proven calibrated fallback.
    public void updateShootingSolution(
        VisionSubsystem visionSubsystem,
        ShootingHelper shootingHelper
    ) {
        if (!isShootingRequestActive) {
            return;
        }

        boolean shouldUseLiveSolution =
            visionSubsystem.hasTarget()
            && visionSubsystem.itsAValidShootingTarget()
            && shootingHelper.isPossibleToShoot();

        if (shouldUseLiveSolution) {
            desiredExitSpeedMetersPerSecond = shootingHelper.getExitSpeedMetersPerSecond();
            desiredHoodAngleRadiansFromModel = shootingHelper.getHoodAngleRadians();
            Logger.recordOutput("Hood/SolutionMode", "LIVE");
            return;
        }

        // Fallback ALWAYS
        desiredExitSpeedMetersPerSecond = ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;
        desiredHoodAngleRadiansFromModel = ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;
        Logger.recordOutput("Hood/SolutionMode", "DEFAULT");
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public void stop() {
        isShootingRequestActive = false;
        hoodOperationalState = HoodOperationalState.STOWED;

        outputs.controlMode = HoodIO.HoodIOControlMode.DISABLED;
        outputs.goalWheelRotationsPerSecond = 0.0;
        outputs.goalHoodAnglePositionRotations = 0.0;
        outputs.indexerEnabled = false;

        readyToShoot = false;
    }

    private boolean isWheelVelocityReady(double goalWheelRotationsPerSecond) {
        double leftMeasuredRotationsPerSecond = inputs.leftWheelVelocityRotationsPerSecond;
        double rightMeasuredRotationsPerSecond = inputs.rightWheelVelocityRotationsPerSecond;

        double leftError = Math.abs(leftMeasuredRotationsPerSecond - goalWheelRotationsPerSecond);
        double rightError = Math.abs(rightMeasuredRotationsPerSecond - (-goalWheelRotationsPerSecond));

        return leftError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS
            && rightError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS;
    }

    private boolean isHoodAngleReady(double goalHoodAnglePositionRotations) {
        double measuredRotations = inputs.hoodAnglePositionRotations;
        double errorRotations = Math.abs(measuredRotations - goalHoodAnglePositionRotations);
        return errorRotations <= HoodConstants.HOOD_ANGLE_TOLERANCE_ROT;
    }

    private boolean areWheelsBelowStopThreshold() {
        double leftAbsoluteRotationsPerSecond = Math.abs(inputs.leftWheelVelocityRotationsPerSecond);
        double rightAbsoluteRotationsPerSecond = Math.abs(inputs.rightWheelVelocityRotationsPerSecond);

        return leftAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND
            && rightAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND;
    }

    public void setShooterProjectileSimulation(ShooterProjectileSimulation newShooterProjectileSimulation) {
        shooterProjectileSimulation =
            (newShooterProjectileSimulation != null) ? newShooterProjectileSimulation : (shotParameters) -> {};
    }

    public void setRobotStateSuppliers(
        Supplier<Pose2d> newRobotPoseFieldSupplier,
        Supplier<ChassisSpeeds> newRobotFieldRelativeChassisSpeedsSupplier
    ) {
        if (newRobotPoseFieldSupplier != null) {
            robotPoseFieldSupplier = newRobotPoseFieldSupplier;
        }
        if (newRobotFieldRelativeChassisSpeedsSupplier != null) {
            robotFieldRelativeChassisSpeedsSupplier = newRobotFieldRelativeChassisSpeedsSupplier;
        }
    }

    private void maybeSimulateShot(boolean indexerEnabledCommanded) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        double currentTimestampSeconds = Timer.getFPGATimestamp();

        boolean isIndexerRisingEdge = indexerEnabledCommanded && !lastIndexerEnabledCommanded;
        lastIndexerEnabledCommanded = indexerEnabledCommanded;

        boolean isAllowedByCooldown = currentTimestampSeconds >= nextAllowedShotTimestampSeconds;

        if (indexerEnabledCommanded && (isIndexerRisingEdge || isAllowedByCooldown)) {
            nextAllowedShotTimestampSeconds = currentTimestampSeconds + SHOT_COOLDOWN_SECONDS;

            ShooterShotParameters shotParameters =
                new ShooterShotParameters(
                    desiredExitSpeedMetersPerSecond,
                    desiredHoodAngleRadiansFromModel,
                    frc.robot.Constants.ShootingConstants.SHOOTER_EXIT_HEIGHT_METERS,
                    robotPoseFieldSupplier.get(),
                    robotFieldRelativeChassisSpeedsSupplier.get()
                );

            shooterProjectileSimulation.simulateShot(shotParameters);
        }
    }
}
