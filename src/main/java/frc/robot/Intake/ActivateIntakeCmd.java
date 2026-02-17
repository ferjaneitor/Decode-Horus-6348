package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ActivateIntakeCmd extends Command {
  private final IntakeSubsystem m_intakeSubsystem;

  public ActivateIntakeCmd(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    // Code to activate the intake mechanism
  }

  @Override
  public void execute() {
    // Code to keep the intake active if necessary
    m_intakeSubsystem.activateIntake();
  }

  @Override
  public void end(boolean interrupted) {
    // Code to stop the intake mechanism if necessary
    m_intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    // Return true when the intake has completed its action, or false if it should run indefinitely
    return true; // Placeholder, change as needed
  }
    
}
