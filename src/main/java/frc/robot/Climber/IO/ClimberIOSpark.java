// File: src/main/java/frc/robot/Climber/ClimberIOSpark.java
package frc.robot.Climber.IO;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Util.SparkUtil;
import static frc.robot.Util.SparkUtil.ifOk;

public final class ClimberIOSpark implements ClimberIO {

    private final SuperSparkMax leftClimberMotorController;
    private final SuperSparkMax rightClimberMotorController;

    private final Debouncer leftClimberConnectedDebouncer =
            new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Debouncer rightClimberConnectedDebouncer =
            new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private ClimberControlMode climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;

    private double climberTargetPositionRotations = 0.0;
    private double climberAppliedVoltageCommandVolts = 0.0;

    public ClimberIOSpark(SuperSparkMax leftClimberMotorController, SuperSparkMax rightClimberMotorController) {
        this.leftClimberMotorController = leftClimberMotorController;
        this.rightClimberMotorController = rightClimberMotorController;
    }

    @Override
    public void updateInputs(ClimberIOInputs climberInputs) {
        // ---------------- Left motor inputs ----------------
        SparkBase leftSparkBase = leftClimberMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(leftSparkBase,
                leftClimberMotorController::getRelativeEncoderPosition,
                (value) -> climberInputs.leftClimberPositionRotations = value);

        ifOk(leftSparkBase,
                leftClimberMotorController::getRelativeEncoderVelocity,
                (value) -> climberInputs.leftClimberVelocityRotationsPerSecond = value);

        ifOk(leftSparkBase,
                new java.util.function.DoubleSupplier[] {leftSparkBase::getAppliedOutput, leftSparkBase::getBusVoltage},
                (values) -> climberInputs.leftClimberAppliedVolts = values[0] * values[1]);

        ifOk(leftSparkBase,
                leftSparkBase::getOutputCurrent,
                (value) -> climberInputs.leftClimberCurrentAmps = value);

        climberInputs.leftClimberConnected =
                leftClimberConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // ---------------- Right motor inputs ----------------
        SparkBase rightSparkBase = rightClimberMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(rightSparkBase,
                rightClimberMotorController::getRelativeEncoderPosition,
                (value) -> climberInputs.rightClimberPositionRotations = value);

        ifOk(rightSparkBase,
                rightClimberMotorController::getRelativeEncoderVelocity,
                (value) -> climberInputs.rightClimberVelocityRotationsPerSecond = value);

        ifOk(rightSparkBase,
                new java.util.function.DoubleSupplier[] {rightSparkBase::getAppliedOutput, rightSparkBase::getBusVoltage},
                (values) -> climberInputs.rightClimberAppliedVolts = values[0] * values[1]);

        ifOk(rightSparkBase,
                rightSparkBase::getOutputCurrent,
                (value) -> climberInputs.rightClimberCurrentAmps = value);

        climberInputs.rightClimberConnected =
                rightClimberConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // ---------------- Control execution (runs every loop) ----------------
        if (climberControlMode == ClimberControlMode.POSITION_PID) {
            leftClimberMotorController.PIDPositionControl(climberTargetPositionRotations);
            rightClimberMotorController.PIDPositionControl(climberTargetPositionRotations);
        } else {
            leftClimberMotorController.setVoltage(climberAppliedVoltageCommandVolts);
            rightClimberMotorController.setVoltage(climberAppliedVoltageCommandVolts);
        }

        // ---------------- Control telemetry ----------------
        climberInputs.climberControlMode = climberControlMode;
        climberInputs.climberTargetPositionRotations = climberTargetPositionRotations;
        climberInputs.climberAppliedVoltageCommandVolts = climberAppliedVoltageCommandVolts;

        climberInputs.leftFeedbackVolts = leftClimberMotorController.getLastFeedbackVolts();
        climberInputs.leftTotalCommandedVolts = leftClimberMotorController.getLastTotalCommandedVolts();

        climberInputs.rightFeedbackVolts = rightClimberMotorController.getLastFeedbackVolts();
        climberInputs.rightTotalCommandedVolts = rightClimberMotorController.getLastTotalCommandedVolts();
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;
        climberAppliedVoltageCommandVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void stopClimber() {
        climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;
        climberAppliedVoltageCommandVolts = 0.0;
    }

    @Override
    public void setClimberPositionPidRotations(double targetPositionRotations) {
        climberControlMode = ClimberControlMode.POSITION_PID;
        climberTargetPositionRotations = targetPositionRotations;
    }
}
