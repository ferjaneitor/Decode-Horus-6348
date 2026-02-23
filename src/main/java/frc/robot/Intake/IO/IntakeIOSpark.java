package frc.robot.Intake.IO;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.filter.Debouncer;

import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Constants.IntakeConstants;
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

        final double[] pivotRawEncoderPositionRotationsHolder = new double[] {0.0};
        final double[] pivotRawEncoderVelocityRotationsPerSecondHolder = new double[] {0.0};

        ifOk(
                pivotSparkBase,
                pivotMotorController::getRelativeEncoderPosition,
                (value) -> {
                    pivotRawEncoderPositionRotationsHolder[0] = value;
                    intakeInputs.pivotRawEncoderPositionRotations = value;
                }
        );

        ifOk(
                pivotSparkBase,
                pivotMotorController::getRelativeEncoderVelocity,
                (value) -> {
                    pivotRawEncoderVelocityRotationsPerSecondHolder[0] = value;
                    intakeInputs.pivotRawEncoderVelocityRotationsPerSecond = value;
                }
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

        // Convert raw encoder rotations -> normalized 0..1 rotations
        intakeInputs.pivotPositionRotations =
                getNormalizedPositionRotationsFromRawEncoderRotations(pivotRawEncoderPositionRotationsHolder[0]);

        intakeInputs.pivotVelocityRotationsPerSecond =
                getNormalizedVelocityRotationsPerSecondFromRawEncoderVelocityRotationsPerSecond(
                        pivotRawEncoderVelocityRotationsPerSecondHolder[0]
                );

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
    public void setPivotRawEncoderPositionRotations(double rawEncoderPositionRotations) {
        pivotMotorController.setRelativeEncoderPositionRotations(rawEncoderPositionRotations);
    }

    @Override
    public void setRunning(boolean runIntake) {
        // no-op in REAL
    }

    @Override
    public boolean isFuelInsideIntake() {
        return false;
    }

    private static double getNormalizedPositionRotationsFromRawEncoderRotations(double rawEncoderRotations) {
        double retractRaw = IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS;
        double deployRaw = IntakeConstants.PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS;

        double denominator = deployRaw - retractRaw;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }

        return (rawEncoderRotations - retractRaw) / denominator;
    }

    private static double getNormalizedVelocityRotationsPerSecondFromRawEncoderVelocityRotationsPerSecond(
            double rawEncoderVelocityRotationsPerSecond
    ) {
        double retractRaw = IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS;
        double deployRaw = IntakeConstants.PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS;

        double denominator = deployRaw - retractRaw;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }

        return rawEncoderVelocityRotationsPerSecond / denominator;
    }
}