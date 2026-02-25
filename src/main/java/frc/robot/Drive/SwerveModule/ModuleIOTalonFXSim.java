package frc.robot.Drive.SwerveModule;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Radians;
import frc.robot.Util.PhoenixUtil;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation swerveModuleSimulation;
  private final double driveRotorToWheelRatio;
  private final double steerRotorToModuleRatio;

  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveModuleSimulation swerveModuleSimulation) {

    super(constants);
    this.swerveModuleSimulation = swerveModuleSimulation;

    // IMPORTANT:
    // Phoenix "GearRatio" values are typically motor_rotations per mechanism_rotation.
    // MapleSim expects rotor_rotations = mechanism_rotations * rotorToMechanismRatio.
    this.driveRotorToWheelRatio = constants.DriveMotorGearRatio;
    this.steerRotorToModuleRatio = constants.SteerMotorGearRatio;

    swerveModuleSimulation.useDriveMotorController(
        new PhoenixUtil.TalonFXMotorControllerSim(super.driveTalon, driveRotorToWheelRatio));

    swerveModuleSimulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            super.turnTalon,
            super.cancoder,
            steerRotorToModuleRatio));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(swerveModuleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angleMeasure -> angleMeasure.in(Radians))
            .toArray();

    Rotation2d[] cachedSteerAngles = swerveModuleSimulation.getCachedSteerAbsolutePositions();
    inputs.odometryTurnPositions = cachedSteerAngles;
  }
}