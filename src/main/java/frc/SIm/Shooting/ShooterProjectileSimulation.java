package frc.SIm.Shooting;

@FunctionalInterface
public interface ShooterProjectileSimulation {
  void simulateShot(ShooterShotParameters shotParameters);
}