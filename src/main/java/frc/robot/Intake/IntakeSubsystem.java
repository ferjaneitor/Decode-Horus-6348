package frc.robot.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    SuperSparkMax intakeSuperSparkMax, pivotIntakeSuperSparkMax;

    boolean itsFullyDeployed, itsFullyRetracted;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    this.intakeSuperSparkMax = new SuperSparkMax(
      IntakeConstants.INTAKE_MOTOR_ID, 
      MotorType.kBrushless,
      IntakeConstants.INTAKE_MOTOR_CONFIG()
    );
    this.pivotIntakeSuperSparkMax = new SuperSparkMax(
      IntakeConstants.PIVOT_INTAKE_MOTOR_ID, 
      MotorType.kBrushless,
      IntakeConstants.PIVOT_INTAKE_MOTOR_CONFIG()
    );

    this.itsFullyDeployed = false;
    this.itsFullyRetracted = true;

  }

  public void activateIntake() {
    this.intakeSuperSparkMax.setVoltage(IntakeConstants.INTAKE_ACTIVATION_VOLTAGE); // Example voltage to activate intake
  }

  public void stopIntake() {
    this.intakeSuperSparkMax.setVoltage(0); // Stop the intake
  } 

  public void deployIntake() {
    if (getIntakePosition() < IntakeConstants.PIVOT_DEPLOY_POSITION_ROT) {
      this.pivotIntakeSuperSparkMax.PIDPositionControl(IntakeConstants.PIVOT_DEPLOY_POSITION_ROT);
    } else {
      itsFullyDeployed = true;
      itsFullyRetracted = false;
    }
  }

  public void retractIntake() {
    if (getIntakePosition() > IntakeConstants.PIVOT_RETRACT_POSITION_ROT) {
      this.pivotIntakeSuperSparkMax.PIDPositionControl(IntakeConstants.PIVOT_RETRACT_POSITION_ROT);
    } else {
      itsFullyDeployed = false;
      itsFullyRetracted = true;
    }
  }

  public void stopPivot() {
    this.pivotIntakeSuperSparkMax.setVoltage(0); // Stop the pivot motor
  }

  public double getIntakePosition() {
    return this.pivotIntakeSuperSparkMax.getRelativeEncoderPosition(); // Get the position of the pivot
  }

  public boolean isIntakeDeployed() {
    return itsFullyDeployed; // Check if the intake is deployed
  }

  public boolean isIntakeRetracted() {
    return itsFullyRetracted; // Check if the intake is retracted
  }
    
}
