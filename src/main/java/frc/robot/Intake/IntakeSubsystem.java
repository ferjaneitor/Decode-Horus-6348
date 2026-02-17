package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.SuperSubsystem.SuperMotors.SparkMax.SuperSparkMax;

public class IntakeSubsystem extends SubsystemBase {

    SuperSparkMax intakeSuperSparkMax, pivotIntakeSuperSparkMax;

    boolean itsFullyDeployed, itsFullyRetracted;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    this.intakeSuperSparkMax = new SuperSparkMax(0, null,null);
    this.pivotIntakeSuperSparkMax = new SuperSparkMax(1, null,null);

    this.itsFullyDeployed = false;
    this.itsFullyRetracted = true;

  }

  public void activateIntake() {
    this.intakeSuperSparkMax.setVoltage(12); // Example voltage to activate intake
  }

  public void stopIntake() {
    this.intakeSuperSparkMax.setVoltage(0); // Stop the intake
  } 

  public void deployIntake() {
    this.pivotIntakeSuperSparkMax.setVoltage(12); // Example voltage to deploy intake
  }

  public void retractIntake() {
    this.pivotIntakeSuperSparkMax.setVoltage(-12); // Example voltage to retract intake
  }

  public double getIntakePosition() {
    return this.pivotIntakeSuperSparkMax.getRelativeEncoderPosition(); // Get the position of the pivot
  }
    
}
