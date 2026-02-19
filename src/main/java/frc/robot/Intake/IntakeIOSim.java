package frc.robot.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.IntakeConstants;

// MapleSim
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import static edu.wpi.first.units.Units.Meters;

public final class IntakeIOSim implements IntakeIO {

    private static final double SIMULATION_TIME_STEP_SECONDS = 0.02;

    private final SingleJointedArmSim pivotArmMechanismSimulation;

    private final IntakeSimulation intakeSimulation;
    private boolean isIntakeSimulationRunning = false;

    private double rollerVoltageCommandVolts = 0.0;
    private double pivotVoltageCommandVolts = 0.0;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.pivotArmMechanismSimulation = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.PIVOT_SIM_GEAR_REDUCTION,
                IntakeConstants.PIVOT_SIM_MOMENT_OF_INERTIA,
                IntakeConstants.PIVOT_SIM_ARM_LENGTH_METERS,
                IntakeConstants.PIVOT_SIM_MIN_ANGLE_RADIANS,
                IntakeConstants.PIVOT_SIM_MAX_ANGLE_RADIANS,
                true,
                IntakeConstants.PIVOT_SIM_START_ANGLE_RADIANS
        );

        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrainSimulation,
                Meters.of(IntakeConstants.MAPLESIM_INTAKE_WIDTH_METERS),
                Meters.of(IntakeConstants.MAPLESIM_INTAKE_EXTENSION_METERS),
                IntakeSimulation.IntakeSide.FRONT,
                IntakeConstants.MAPLESIM_INTAKE_CAPACITY
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs intakeInputs) {
        double batteryBusVoltageVolts = RobotController.getBatteryVoltage();

        double clampedPivotVoltageVolts =
                MathUtil.clamp(pivotVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

        pivotArmMechanismSimulation.setInputVoltage(clampedPivotVoltageVolts);
        pivotArmMechanismSimulation.update(SIMULATION_TIME_STEP_SECONDS);

        double clampedRollerVoltageVolts =
                MathUtil.clamp(rollerVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

        double rollerVelocityRotationsPerSecond =
                (Math.abs(clampedRollerVoltageVolts) < 1e-6)
                        ? 0.0
                        : (clampedRollerVoltageVolts / batteryBusVoltageVolts) * 50.0;

        double rollerCurrentAmps =
                (Math.abs(clampedRollerVoltageVolts) < 1e-6)
                        ? 0.0
                        : 10.0 * Math.abs(clampedRollerVoltageVolts / batteryBusVoltageVolts);

        intakeInputs.rollerConnected = true;
        intakeInputs.rollerVelocityRotationsPerSecond = rollerVelocityRotationsPerSecond;
        intakeInputs.rollerAppliedVolts = clampedRollerVoltageVolts;
        intakeInputs.rollerCurrentAmps = rollerCurrentAmps;

        intakeInputs.pivotConnected = true;
        intakeInputs.pivotPositionRotations = pivotArmMechanismSimulation.getAngleRads() / (2.0 * Math.PI);
        intakeInputs.pivotVelocityRotationsPerSecond = pivotArmMechanismSimulation.getVelocityRadPerSec() / (2.0 * Math.PI);
        intakeInputs.pivotAppliedVolts = clampedPivotVoltageVolts;
        intakeInputs.pivotCurrentAmps = Math.abs(pivotArmMechanismSimulation.getCurrentDrawAmps());

        int fuelCountInsideIntake = intakeSimulation.getGamePiecesAmount();
        intakeInputs.fuelCountInsideIntake = fuelCountInsideIntake;
        intakeInputs.isFuelInsideIntake = fuelCountInsideIntake != 0;
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerVoltageCommandVolts = voltage;
    }

    @Override
    public void stopRoller() {
        rollerVoltageCommandVolts = 0.0;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotVoltageCommandVolts = voltage;
    }

    @Override
    public void stopPivot() {
        pivotVoltageCommandVolts = 0.0;
    }

    @Override
    public void setRunning(boolean runIntake) {
        if (runIntake && !isIntakeSimulationRunning) {
            intakeSimulation.startIntake();
            isIntakeSimulationRunning = true;
            return;
        }

        if (!runIntake && isIntakeSimulationRunning) {
            intakeSimulation.stopIntake();
            isIntakeSimulationRunning = false;
        }
    }

    @Override
    public boolean isFuelInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }
}
