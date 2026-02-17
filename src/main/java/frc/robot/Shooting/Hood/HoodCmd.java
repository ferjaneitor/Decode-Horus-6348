package frc.robot.Shooting.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
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
    public void execute() {
        boolean shouldUseLiveSolution =
            visionSubsystem.hasTarget()
            && visionSubsystem.itsAValidShootingTarget()
            && shootingHelper.isPossibleToShoot();

        double desiredExitSpeedMetersPerSecond = shouldUseLiveSolution
            ? shootingHelper.getExitSpeedMetersPerSecond()
            : ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;

        double desiredHoodAngleRadians = shouldUseLiveSolution
            ? shootingHelper.getHoodAngleRadians()
            : ShootingConstants.DEFAULT_HOOD_ANGLE_ROT;

        hoodSubsystem.setGoalsMetersPerSecondAndRadians(
            desiredExitSpeedMetersPerSecond,
            desiredHoodAngleRadians
        );

        hoodSubsystem.setIndexerEnabled(hoodSubsystem.isReadyToShoot());
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}