package frc.robot.Intake;

import com.revrobotics.spark.SparkBase;
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

    public IntakeIOSpark(SuperSparkMax rollerMotorController, SuperSparkMax pivotMotorController) {
        this.rollerMotorController = rollerMotorController;
        this.pivotMotorController = pivotMotorController;
    }

    @Override
    public void updateInputs(IntakeIOInputs intakeInputs) {
        // Roller
        SparkBase rollerSparkBase = rollerMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(
                rollerSparkBase,
                rollerMotorController::getRelativeEncoderVelocity,
                (value) -> intakeInputs.rollerVelocityRotationsPerSecond = value
        );

        ifOk(
                rollerSparkBase,
                new java.util.function.DoubleSupplier[] {rollerSparkBase::getAppliedOutput, rollerSparkBase::getBusVoltage},
                (values) -> intakeInputs.rollerAppliedVolts = values[0] * values[1]
        );

        ifOk(
                rollerSparkBase,
                rollerSparkBase::getOutputCurrent,
                (value) -> intakeInputs.rollerCurrentAmps = value
        );

        intakeInputs.rollerConnected = rollerConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // Pivot
        SparkBase pivotSparkBase = pivotMotorController.getSparkBase();

        SparkUtil.sparkStickyFault = false;
        ifOk(
                pivotSparkBase,
                pivotMotorController::getRelativeEncoderPosition,
                (value) -> intakeInputs.pivotPositionRotations = value
        );

        ifOk(
                pivotSparkBase,
                pivotMotorController::getRelativeEncoderVelocity,
                (value) -> intakeInputs.pivotVelocityRotationsPerSecond = value
        );

        ifOk(
                pivotSparkBase,
                new java.util.function.DoubleSupplier[] {pivotSparkBase::getAppliedOutput, pivotSparkBase::getBusVoltage},
                (values) -> intakeInputs.pivotAppliedVolts = values[0] * values[1]
        );

        ifOk(
                pivotSparkBase,
                pivotSparkBase::getOutputCurrent,
                (value) -> intakeInputs.pivotCurrentAmps = value
        );

        intakeInputs.pivotConnected = pivotConnectedDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // MapleSim fields (REAL)
        intakeInputs.isFuelInsideIntake = false;
        intakeInputs.fuelCountInsideIntake = 0;
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
        pivotMotorController.setVoltage(voltage);
    }

    @Override
    public void stopPivot() {
        pivotMotorController.setVoltage(0.0);
    }

    @Override
    public void setRunning(boolean runIntake) {
        // no-op en REAL
    }

    @Override
    public boolean isFuelInsideIntake() {
        return false;
    }
}
