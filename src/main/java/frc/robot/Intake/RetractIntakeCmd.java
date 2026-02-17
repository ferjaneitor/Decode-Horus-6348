package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class RetractIntakeCmd extends Command {
  private final IntakeSubsystem m_intakeSubsystem;

  public RetractIntakeCmd(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    // Code to retract the intake mechanism
  }

  @Override
  public void execute() {
    // Code to keep the intake retracted if necessary
  }

  @Override
  public void end(boolean interrupted) {
    // Code to stop the intake mechanism if necessary
  }

  @Override
  public boolean isFinished() {
    // Return true when the intake is fully retracted, or false if it should run indefinitely
    return true; // Placeholder, change as needed
  }
    
}
