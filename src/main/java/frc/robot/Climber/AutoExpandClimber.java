package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoExpandClimber extends Command {
    private final ClimberSubsystem climberSubsystem;

    public AutoExpandClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

     @Override
  public void initialize() {
    // Code to deploy the intake mechanism
  }

  @Override
    public void execute() {
        // Code to keep the intake deployed if necessary
        if (!climberSubsystem.isClimberExtended()) {
            climberSubsystem.AutoExpand();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the intake mechanism if necessary
        climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Return true when the intake is fully deployed, or false if it should run indefinitely
        return climberSubsystem.isClimberExtended(); // Check if the intake is fully deployed
    }
    
}
