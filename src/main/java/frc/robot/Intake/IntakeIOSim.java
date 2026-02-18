package frc.robot.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.IntakeConstants;

/**
 * Simple sim implementation for IntakeIO.
 *
 * Goal:
 * - Keep it lightweight and stable for AdvantageScope testing.
 * - Pivot is simulated with SingleJointedArmSim + simple PID.
 * - Roller is NOT physically simulated (no FlywheelSim / DCMotorSim).
 *   It only reports voltage/current/velocity in a consistent "fake" way.
 */
public final class IntakeIOSim implements IntakeIO {

    private static final double SIMULATION_TIME_STEP_SECONDS = 0.02;

    // Pivot mechanism simulation (lightweight, stable)
    private final SingleJointedArmSim pivotArmMechanismSimulation;
    private final PIDController pivotPositionController;

    // Commands
    private double rollerVoltageCommandVolts = 0.0;
    private double pivotVoltageCommandVolts = 0.0;

    private PivotControlMode pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
    private double pivotTargetPositionRotations = 0.0;

    // Telemetry
    private double lastPivotFeedbackVolts = 0.0;
    private double lastPivotTotalCommandedVolts = 0.0;

    public IntakeIOSim() {
        pivotArmMechanismSimulation = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.PIVOT_SIM_GEAR_REDUCTION,
                IntakeConstants.PIVOT_SIM_MOMENT_OF_INERTIA,
                IntakeConstants.PIVOT_SIM_ARM_LENGTH_METERS,
                IntakeConstants.PIVOT_SIM_MIN_ANGLE_RADIANS,
                IntakeConstants.PIVOT_SIM_MAX_ANGLE_RADIANS,
                true,
                IntakeConstants.PIVOT_SIM_START_ANGLE_RADIANS
        );

        pivotPositionController = new PIDController(
                IntakeConstants.PIVOT_SIM_PID_P,
                IntakeConstants.PIVOT_SIM_PID_I,
                IntakeConstants.PIVOT_SIM_PID_D
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs intakeInputs) {
        double batteryBusVoltageVolts = RobotController.getBatteryVoltage();

        // ---------------- Pivot control ----------------
        if (pivotControlMode == PivotControlMode.POSITION_PID) {
            double measuredAngleRadians = pivotArmMechanismSimulation.getAngleRads();
            double targetAngleRadians = pivotTargetPositionRotations * 2.0 * Math.PI;

            lastPivotFeedbackVolts =
                    pivotPositionController.calculate(measuredAngleRadians, targetAngleRadians);

            lastPivotTotalCommandedVolts =
                    MathUtil.clamp(lastPivotFeedbackVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            pivotVoltageCommandVolts = lastPivotTotalCommandedVolts;
        } else {
            pivotPositionController.reset();
            lastPivotFeedbackVolts = 0.0;

            pivotVoltageCommandVolts =
                    MathUtil.clamp(pivotVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            lastPivotTotalCommandedVolts = pivotVoltageCommandVolts;
        }

        // ---------------- Update mechanism simulation ----------------
        pivotArmMechanismSimulation.setInputVoltage(pivotVoltageCommandVolts);
        pivotArmMechanismSimulation.update(SIMULATION_TIME_STEP_SECONDS);

        // ---------------- Roller "fake" sim (no physics) ----------------
        // For AdvantageScope testing, roller velocity/current being "reasonable" is enough.
        // If you prefer, set these to 0.0 always and only keep applied volts.
        double rollerAppliedVoltsClamped =
                MathUtil.clamp(rollerVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

        // Very lightweight heuristics (optional)
        double rollerVelocityRotationsPerSecond =
                (Math.abs(rollerAppliedVoltsClamped) < 1e-6) ? 0.0 : (rollerAppliedVoltsClamped / batteryBusVoltageVolts) * 50.0;
        double rollerCurrentAmps =
                (Math.abs(rollerAppliedVoltsClamped) < 1e-6) ? 0.0 : 10.0 * Math.abs(rollerAppliedVoltsClamped / batteryBusVoltageVolts);

        // ---------------- Fill inputs ----------------
        intakeInputs.rollerConnected = true;
        intakeInputs.rollerVelocityRotationsPerSecond = rollerVelocityRotationsPerSecond;
        intakeInputs.rollerAppliedVolts = rollerAppliedVoltsClamped;
        intakeInputs.rollerCurrentAmps = rollerCurrentAmps;

        intakeInputs.pivotConnected = true;
        intakeInputs.pivotPositionRotations = pivotArmMechanismSimulation.getAngleRads() / (2.0 * Math.PI);
        intakeInputs.pivotVelocityRotationsPerSecond = pivotArmMechanismSimulation.getVelocityRadPerSec() / (2.0 * Math.PI);
        intakeInputs.pivotAppliedVolts = pivotVoltageCommandVolts;
        intakeInputs.pivotCurrentAmps = Math.abs(pivotArmMechanismSimulation.getCurrentDrawAmps());

        intakeInputs.pivotControlMode = pivotControlMode;
        intakeInputs.pivotTargetPositionRotations = pivotTargetPositionRotations;

        // These fields exist in your updated inputs structure
        intakeInputs.feedbackVolts = lastPivotFeedbackVolts;
        intakeInputs.totalCommandedVolts = lastPivotTotalCommandedVolts;

        // If you still have this field in your IntakeIOInputs, keep it; if not, delete this line.
        // intakeInputs.pivotPositionErrorRotations = pivotTargetPositionRotations - intakeInputs.pivotPositionRotations;
    }

    // ---------------- Roller commands ----------------

    @Override
    public void setRollerVoltage(double voltage) {
        rollerVoltageCommandVolts = voltage;
    }

    @Override
    public void stopRoller() {
        rollerVoltageCommandVolts = 0.0;
    }

    // ---------------- Pivot commands ----------------

    @Override
    public void setPivotVoltage(double voltage) {
        pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
        pivotVoltageCommandVolts = voltage;
    }

    @Override
    public void stopPivot() {
        pivotControlMode = PivotControlMode.OPEN_LOOP_VOLTAGE;
        pivotVoltageCommandVolts = 0.0;
    }

    @Override
    public void setPivotPositionPidRotations(double targetPositionRotations) {
        pivotControlMode = PivotControlMode.POSITION_PID;
        pivotTargetPositionRotations = targetPositionRotations;
    }
}
