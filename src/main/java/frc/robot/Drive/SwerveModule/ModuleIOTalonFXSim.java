package frc.robot.Drive.SwerveModule;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import frc.robot.Util.PhoenixUtil;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation swerveModuleSimulation;

  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveModuleSimulation swerveModuleSimulation) {

    super(constants);
    this.swerveModuleSimulation = swerveModuleSimulation;

    // Attach motor-controller sims to MapleSim module sim
    swerveModuleSimulation.useDriveMotorController(
        new PhoenixUtil.TalonFXMotorControllerSim(super.driveTalon));

    swerveModuleSimulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(super.turnTalon, super.cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // High-rate odometry samples (matches MapleSim sub-ticks)
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(swerveModuleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angleMeasure -> angleMeasure.in(Radians))
            .toArray();

    Rotation2d[] cachedSteerAngles = swerveModuleSimulation.getCachedSteerAbsolutePositions();
    inputs.odometryTurnPositions = cachedSteerAngles;
  }
}
