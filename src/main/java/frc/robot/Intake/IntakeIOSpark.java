package frc.robot.Intake;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;

import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Util.SparkUtil;
import static frc.robot.Util.SparkUtil.ifOk;

public final class IntakeIOSpark implements IntakeIO {

    private final SuperSparkMax rollerMotorController;
    private final SuperSparkMax pivotMotorController;

    private final Debouncer rollerConnectedDebouncer =
            new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Debouncer pivotConnectedDebouncer =
            new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private PivotControlMode pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
    private double pivotTargetPositionRotations = 0.0;
    private double pivotAppliedVoltageCommand = 0.0;

    public IntakeIOSpark(SuperSparkMax rollerMotorController, SuperSparkMax pivotMotorController) {
        this.rollerMotorController = rollerMotorController;
        this.pivotMotorController = pivotMotorController;
    }

    @Override
    public void updateInputs(IntakeIOInputs intakeInputs) {
        // -------- Roller inputs --------
        SparkBase rollerSparkBase = rollerMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(rollerSparkBase,
                rollerMotorController::getRelativeEncoderVelocity,
                (value) -> intakeInputs.rollerVelocityRotationsPerSecond = value);

        ifOk(rollerSparkBase,
                new java.util.function.DoubleSupplier[] {rollerSparkBase::getAppliedOutput, rollerSparkBase::getBusVoltage},
                (values) -> intakeInputs.rollerAppliedVolts = values[0] * values[1]);

        ifOk(rollerSparkBase,
                rollerSparkBase::getOutputCurrent,
                (value) -> intakeInputs.rollerCurrentAmps = value);

        intakeInputs.rollerConnected = rollerConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // -------- Pivot inputs --------
        SparkBase pivotSparkBase = pivotMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(pivotSparkBase,
                pivotMotorController::getRelativeEncoderPosition,
                (value) -> intakeInputs.pivotPositionRotations = value);

        ifOk(pivotSparkBase,
                pivotMotorController::getRelativeEncoderVelocity,
                (value) -> intakeInputs.pivotVelocityRotationsPerSecond = value);

        ifOk(pivotSparkBase,
                new java.util.function.DoubleSupplier[] {pivotSparkBase::getAppliedOutput, pivotSparkBase::getBusVoltage},
                (values) -> intakeInputs.pivotAppliedVolts = values[0] * values[1]);

        ifOk(pivotSparkBase,
                pivotSparkBase::getOutputCurrent,
                (value) -> intakeInputs.pivotCurrentAmps = value);

        intakeInputs.pivotConnected = pivotConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // -------- Pivot control (PID simple) --------
        if (pivotControlMode == PivotControlMode.POSITION_PID) {
            pivotMotorController.PIDPositionControl(pivotTargetPositionRotations);
        } else {
            pivotMotorController.setVoltage(pivotAppliedVoltageCommand);
        }

        // -------- Control telemetry --------
        intakeInputs.pivotControlMode = pivotControlMode;
        intakeInputs.pivotTargetPositionRotations = pivotTargetPositionRotations;

        double positionErrorRotations = intakeInputs.pivotTargetPositionRotations - intakeInputs.pivotPositionRotations;
        intakeInputs.pivotPositionErrorRotations = positionErrorRotations;

        intakeInputs.feedbackVolts = pivotMotorController.getLastFeedbackVolts();
        intakeInputs.totalCommandedVolts = pivotMotorController.getLastTotalCommandedVolts();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotorController.setVoltage(voltage);
    }

    @Override
    public void stopRoller() {
        rollerMotorController.setVoltage(0.0);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
        pivotAppliedVoltageCommand = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void stopPivot() {
        pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
        pivotAppliedVoltageCommand = 0.0;
    }

    @Override
    public void setPivotPositionPidRotations(double targetPositionRotations) {
        pivotControlMode = PivotControlMode.POSITION_PID;
        pivotTargetPositionRotations = targetPositionRotations;
    }
}
