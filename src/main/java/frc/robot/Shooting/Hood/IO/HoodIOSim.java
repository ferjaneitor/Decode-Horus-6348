package frc.robot.Shooting.Hood.IO;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants.HoodConstants;
import frc.robot.Drive.Generated.TunerConstants;

public class HoodIOSim implements HoodIO {

    private static final double SIMULATION_TIME_STEP_SECONDS = 0.02;

    // TalonFX hardware objects (exist in sim too)
    private final TalonFX hoodAngleMotorTalonFx =
        new TalonFX(HoodConstants.HOOD_ANGLE_TALON_ID, TunerConstants.kCANBus);
    private final TalonFX rightWheelMotorTalonFx =
        new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, TunerConstants.kCANBus);
    private final TalonFX leftWheelMotorTalonFx =
        new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID, TunerConstants.kCANBus);
    private final TalonFX indexerMotorTalonFx =
        new TalonFX(HoodConstants.INDEXER_TALON_ID, TunerConstants.kCANBus);

    // Phoenix 6 sim states
    private final TalonFXSimState hoodAngleMotorSimState = hoodAngleMotorTalonFx.getSimState();
    private final TalonFXSimState rightWheelMotorSimState = rightWheelMotorTalonFx.getSimState();
    private final TalonFXSimState leftWheelMotorSimState = leftWheelMotorTalonFx.getSimState();
    private final TalonFXSimState indexerMotorSimState = indexerMotorTalonFx.getSimState();

    // Control requests (same as real)
    private final VelocityTorqueCurrentFOC rightWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC leftWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final MotionMagicExpoTorqueCurrentFOC hoodAnglePositionRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final DutyCycleOut indexerDutyCycleRequest = new DutyCycleOut(0.0);

    // Physics sims (these just turn motor voltage into motion)
    private final DCMotor wheelMotorModel = DCMotor.getFalcon500(1);
    private final DCMotorSim leftWheelPhysicsSimulation =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(wheelMotorModel, 0.001, 1.0), wheelMotorModel);
    private final DCMotorSim rightWheelPhysicsSimulation =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(wheelMotorModel, 0.001, 1.0), wheelMotorModel);

    private final DCMotor hoodMotorModel = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim hoodAnglePhysicsSimulation =
        new SingleJointedArmSim(
            hoodMotorModel,
            1.0,
            0.004,
            0.33,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(90.0),
            false,
            0.0
        );

    private HoodIOControlMode lastControlMode = HoodIOControlMode.DISABLED;
    private double goalWheelRotationsPerSecond = 0.0;
    private double goalHoodAnglePositionRotations = 0.0;
    private boolean indexerEnabled = false;

    public HoodIOSim() {
        // In sim you can still apply the same configs if you want.
        // If you already applied configs in HoodIOTalonFx, you can keep sim minimal here.
        // But if your sim behavior depends on configs (Motion Magic, etc.), then apply them here too.
        hoodAngleMotorTalonFx.getConfigurator().apply(HoodConstants.HOOD_ANGLE_SLOT_CONFIGS);
        rightWheelMotorTalonFx.getConfigurator().apply(HoodConstants.RIGHT_HOOD_PROPULSION_SLOT_CONFIGS);
        leftWheelMotorTalonFx.getConfigurator().apply(HoodConstants.LEFT_HOOD_PROPULSION_SLOT_CONFIGS);
        indexerMotorTalonFx.getConfigurator().apply(HoodConstants.INDEXER_SLOT_CONFIGS);
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        lastControlMode = outputs.controlMode;
        goalWheelRotationsPerSecond = outputs.goalWheelRotationsPerSecond;
        goalHoodAnglePositionRotations = outputs.goalHoodAnglePositionRotations;
        indexerEnabled = outputs.indexerEnabled;

        if (lastControlMode == HoodIOControlMode.DISABLED) {
            rightWheelMotorTalonFx.setControl(rightWheelVelocityRequest.withVelocity(0.0));
            leftWheelMotorTalonFx.setControl(leftWheelVelocityRequest.withVelocity(0.0));
            hoodAngleMotorTalonFx.setControl(hoodAnglePositionRequest.withPosition(hoodAngleMotorTalonFx.getPosition().getValueAsDouble()));
            indexerMotorTalonFx.setControl(indexerDutyCycleRequest.withOutput(0.0));
            return;
        }

        // Same sign convention as your real IO
        rightWheelMotorTalonFx.setControl(rightWheelVelocityRequest.withVelocity(-goalWheelRotationsPerSecond));
        leftWheelMotorTalonFx.setControl(leftWheelVelocityRequest.withVelocity(goalWheelRotationsPerSecond));
        hoodAngleMotorTalonFx.setControl(hoodAnglePositionRequest.withPosition(goalHoodAnglePositionRotations));

        double requestedDutyCycle = indexerEnabled ? HoodConstants.INDEXER_SPEED : 0.0;
        indexerMotorTalonFx.setControl(indexerDutyCycleRequest.withOutput(requestedDutyCycle));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        // Phoenix sim needs supply voltage fed in
        hoodAngleMotorSimState.setSupplyVoltage(batteryVoltage);
        rightWheelMotorSimState.setSupplyVoltage(batteryVoltage);
        leftWheelMotorSimState.setSupplyVoltage(batteryVoltage);
        indexerMotorSimState.setSupplyVoltage(batteryVoltage);

        // Read what voltage Phoenix is actually applying (output of its internal control)
        double leftWheelAppliedVoltage = leftWheelMotorSimState.getMotorVoltage();
        double rightWheelAppliedVoltage = rightWheelMotorSimState.getMotorVoltage();
        double hoodAngleAppliedVoltage = hoodAngleMotorSimState.getMotorVoltage();
        double indexerAppliedVoltage = indexerMotorSimState.getMotorVoltage();

        // Run physics sims using those voltages
        leftWheelPhysicsSimulation.setInputVoltage(MathUtil.clamp(leftWheelAppliedVoltage, -12.0, 12.0));
        rightWheelPhysicsSimulation.setInputVoltage(MathUtil.clamp(rightWheelAppliedVoltage, -12.0, 12.0));
        hoodAnglePhysicsSimulation.setInputVoltage(MathUtil.clamp(hoodAngleAppliedVoltage, -12.0, 12.0));

        leftWheelPhysicsSimulation.update(SIMULATION_TIME_STEP_SECONDS);
        rightWheelPhysicsSimulation.update(SIMULATION_TIME_STEP_SECONDS);
        hoodAnglePhysicsSimulation.update(SIMULATION_TIME_STEP_SECONDS);

        // Convert physics state back into rotor units for TalonFX sim state
        double leftWheelRotorVelocityRotationsPerSecond =
            radiansPerSecondToRotationsPerSecond(leftWheelPhysicsSimulation.getAngularVelocityRadPerSec());
        double rightWheelRotorVelocityRotationsPerSecond =
            radiansPerSecondToRotationsPerSecond(rightWheelPhysicsSimulation.getAngularVelocityRadPerSec());

        double leftWheelRotorPositionRotations =
            radiansToRotations(leftWheelPhysicsSimulation.getAngularPositionRad());
        double rightWheelRotorPositionRotations =
            radiansToRotations(rightWheelPhysicsSimulation.getAngularPositionRad());

        // Hood: physics sim is in radians; your sensor units are rotations.
        double hoodAngleRotorPositionRotations =
            hoodAnglePhysicsSimulation.getAngleRads() * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;
        double hoodAngleRotorVelocityRotationsPerSecond =
            hoodAnglePhysicsSimulation.getVelocityRadPerSec() * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

        // Push back into CTRE sim state (this is what makes TalonFX signals match)
        leftWheelMotorSimState.setRotorVelocity(leftWheelRotorVelocityRotationsPerSecond);
        leftWheelMotorSimState.setRawRotorPosition(leftWheelRotorPositionRotations);

        rightWheelMotorSimState.setRotorVelocity(rightWheelRotorVelocityRotationsPerSecond);
        rightWheelMotorSimState.setRawRotorPosition(rightWheelRotorPositionRotations);

        hoodAngleMotorSimState.setRotorVelocity(hoodAngleRotorVelocityRotationsPerSecond);
        hoodAngleMotorSimState.setRawRotorPosition(hoodAngleRotorPositionRotations);

        // Indexer: simple model, we don’t simulate mechanics; just expose voltage.
        // If you want, you can add a small DCMotorSim for indexer too.
        indexerMotorSimState.setRotorVelocity(0.0);

        // Fill inputs (use sim state + physics; this avoids stale status signal timing issues)
        inputs.hoodAngleMotorConnected = true;
        inputs.leftWheelMotorConnected = true;
        inputs.rightWheelMotorConnected = true;
        inputs.indexerMotorConnected = true;

        inputs.leftWheelVelocityRotationsPerSecond = leftWheelRotorVelocityRotationsPerSecond;
        inputs.rightWheelVelocityRotationsPerSecond = rightWheelRotorVelocityRotationsPerSecond;

        inputs.hoodAnglePositionRotations = hoodAngleRotorPositionRotations;
        inputs.hoodAngleVelocityRotationsPerSecond = hoodAngleRotorVelocityRotationsPerSecond;

        inputs.leftWheelAppliedVolts = leftWheelAppliedVoltage;
        inputs.rightWheelAppliedVolts = rightWheelAppliedVoltage;
        inputs.hoodAngleAppliedVolts = hoodAngleAppliedVoltage;
        inputs.indexerAppliedVolts = indexerAppliedVoltage;

        // Phoenix sim auto-calculates these; pull from sim state when available
        inputs.leftWheelSupplyCurrentAmps = leftWheelMotorSimState.getSupplyCurrent();
        inputs.rightWheelSupplyCurrentAmps = rightWheelMotorSimState.getSupplyCurrent();
        inputs.hoodAngleSupplyCurrentAmps = hoodAngleMotorSimState.getSupplyCurrent();
        inputs.indexerSupplyCurrentAmps = indexerMotorSimState.getSupplyCurrent();

        inputs.leftWheelTorqueCurrentAmps = leftWheelMotorSimState.getTorqueCurrent();
        inputs.rightWheelTorqueCurrentAmps = rightWheelMotorSimState.getTorqueCurrent();
        inputs.hoodAngleTorqueCurrentAmps = hoodAngleMotorSimState.getTorqueCurrent();
        inputs.indexerTorqueCurrentAmps = indexerMotorSimState.getTorqueCurrent();

        inputs.leftWheelTempCelsius = 0.0;
        inputs.rightWheelTempCelsius = 0.0;
        inputs.hoodAngleTempCelsius = 0.0;
        inputs.indexerTempCelsius = 0.0;
    }

    private double radiansPerSecondToRotationsPerSecond(double radiansPerSecond) {
        return radiansPerSecond / (2.0 * Math.PI);
    }

    private double radiansToRotations(double radians) {
        return radians / (2.0 * Math.PI);
    }
}
