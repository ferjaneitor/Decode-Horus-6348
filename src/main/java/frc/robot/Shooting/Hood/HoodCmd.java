package frc.robot.Shooting.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Vision.VisionSubsystem;

public class HoodCmd extends Command {

    private final HoodSubsystem hoodSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ShootingHelper shootingHelper;

    public HoodCmd(
        HoodSubsystem hoodSubsystem,
        VisionSubsystem visionSubsystem,
        ShootingHelper shootingHelper
    ) {
        this.hoodSubsystem = hoodSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.shootingHelper = shootingHelper;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        hoodSubsystem.setShootingRequestActive(true);
    }

    @Override
    public void execute() {
        hoodSubsystem.updateShootingSolution(visionSubsystem, shootingHelper);
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.setShootingRequestActive(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
