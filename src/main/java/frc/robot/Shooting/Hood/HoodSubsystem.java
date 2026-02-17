package frc.robot.Shooting.Hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.AutoLogger.HoodIOInputsAutoLogged;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final HoodIO hoodIo;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO.HoodIOOutputs outputs = new HoodIO.HoodIOOutputs();

    private final Debouncer readyDebouncerSeconds =
        new Debouncer(0.12, Debouncer.DebounceType.kRising);

    private double goalExitSpeedMetersPerSecond = 0.0;
    private double goalHoodAngleRadiansFromModel = 0.0;

    @AutoLogOutput(key = "Hood/ReadyToShoot")
    private boolean readyToShoot = false;

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

        outputs.goalWheelRotationsPerSecond =
            goalExitSpeedMetersPerSecond * HoodConstants.CONVERSION_RATIO_FROM_METERS_TO_RPS;

        double complementaryAngleRadians =
            HoodConstants.COMPLEMENTARY_ANGLE - goalHoodAngleRadiansFromModel;

        outputs.goalHoodAnglePositionRotations =
            complementaryAngleRadians * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

        boolean rawReady =
            isWheelVelocityReady(outputs.goalWheelRotationsPerSecond) &&
            isHoodAngleReady(outputs.goalHoodAnglePositionRotations);

        readyToShoot = readyDebouncerSeconds.calculate(rawReady);

        Logger.recordOutput("Hood/GoalExitSpeedMetersPerSecond", goalExitSpeedMetersPerSecond);
        Logger.recordOutput("Hood/GoalHoodAngleRadiansFromModel", goalHoodAngleRadiansFromModel);
        Logger.recordOutput("Hood/GoalWheelRotationsPerSecond", outputs.goalWheelRotationsPerSecond);
        Logger.recordOutput("Hood/GoalHoodAnglePositionRotations", outputs.goalHoodAnglePositionRotations);
        Logger.recordOutput("Hood/RawReadyToShoot", rawReady);
        Logger.recordOutput("Hood/ControlMode", outputs.controlMode.toString());
        Logger.recordOutput("Hood/IndexerEnabled", outputs.indexerEnabled);

        hoodIo.applyOutputs(outputs);
    }

    public void setGoalsMetersPerSecondAndRadians(double desiredExitSpeedMetersPerSecond, double desiredHoodAngleRadians) {
        goalExitSpeedMetersPerSecond = desiredExitSpeedMetersPerSecond;
        goalHoodAngleRadiansFromModel = desiredHoodAngleRadians;
    }

    public void setIndexerEnabled(boolean enabled) {
        outputs.indexerEnabled = enabled;
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public void stop() {
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
}
