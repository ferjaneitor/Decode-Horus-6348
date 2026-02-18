package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployIntakeCmd extends Command {
  private final IntakeSubsystem m_intakeSubsystem;

  public DeployIntakeCmd(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    // Code to deploy the intake mechanism
  }

  @Override
  public void execute() {
    // Code to keep the intake deployed if necessary
    if (!m_intakeSubsystem.isIntakeDeployed()) {
      m_intakeSubsystem.deployIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Code to stop the intake mechanism if necessary
    m_intakeSubsystem.stopPivot();
  }

  @Override
  public boolean isFinished() {
    // Return true when the intake is fully deployed, or false if it should run indefinitely
    return m_intakeSubsystem.isIntakeDeployed(); // Check if the intake is fully deployed
  }
    
}
