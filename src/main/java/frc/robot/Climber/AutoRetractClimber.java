package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoRetractClimber extends Command {
    private final ClimberSubsystem climberSubsystem;

    public AutoRetractClimber(ClimberSubsystem climberSubsystem) {
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
        if (!climberSubsystem.isClimberRetracted()) {
            climberSubsystem.AutoRetract();
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
        return climberSubsystem.isClimberRetracted(); // Check if the intake is fully deployed
    }
    
}
