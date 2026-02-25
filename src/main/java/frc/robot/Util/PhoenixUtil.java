package frc.robot.Util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public final class PhoenixUtil {
    private PhoenixUtil() {}

    public static void tryUntilOk(int attemptCount, Supplier<StatusCode> commandSupplier) {
        for (int attemptIndex = 0; attemptIndex < attemptCount; attemptIndex++) {
            StatusCode statusCode = commandSupplier.get();
            if (statusCode.isOK()) {
                return;
            }
        }
    }

    public static double[] getSimulationOdometryTimeStamps() {
        int simulationSubTickCount = SimulatedArena.getSimulationSubTicksIn1Period();
        double currentTimestampSeconds = RobotController.getFPGATime() / 1e6;

        if (simulationSubTickCount <= 0) {
            return new double[] { currentTimestampSeconds };
        }

        double robotPeriodSeconds = SimulatedArena.getSimulationDt().in(Seconds);
        double subTickPeriodSeconds = robotPeriodSeconds / simulationSubTickCount;

        double[] timestampsSeconds = new double[simulationSubTickCount];
        for (int subTickIndex = 0; subTickIndex < simulationSubTickCount; subTickIndex++) {
            int reversedIndex = (simulationSubTickCount - 1) - subTickIndex;
            timestampsSeconds[subTickIndex] =
                currentTimestampSeconds - (reversedIndex * subTickPeriodSeconds);
        }

        return timestampsSeconds;
    }

    /**
     * MapleSim motor-controller adapter for a TalonFX.
     *
     * rotorToMechanismRatio:
     *   rotor_rotations = mechanism_rotations * rotorToMechanismRatio
     *
     * For swerve drive motor:
     *   mechanism_rotations = wheel_rotations
     *   rotorToMechanismRatio = DriveMotorGearRatio
     *
     * For swerve steer motor:
     *   mechanism_rotations = module_steer_rotations
     *   rotorToMechanismRatio = SteerMotorGearRatio
     */
    public static final class TalonFXMotorControllerSim implements SimulatedMotorController {
        private final TalonFXSimState talonFxSimState;
        private final double rotorToMechanismRatio;

        public TalonFXMotorControllerSim(TalonFX talonFx, double rotorToMechanismRatio) {
            this.talonFxSimState = talonFx.getSimState();
            this.rotorToMechanismRatio = rotorToMechanismRatio;
        }

        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            talonFxSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            double mechanismPositionRotations = mechanismAngle.in(Radians) / (2.0 * Math.PI);
            double mechanismVelocityRotationsPerSecond =
                mechanismVelocity.in(RadiansPerSecond) / (2.0 * Math.PI);

            double rotorPositionRotations = mechanismPositionRotations * rotorToMechanismRatio;
            double rotorVelocityRotationsPerSecond =
                mechanismVelocityRotationsPerSecond * rotorToMechanismRatio;

            talonFxSimState.setRawRotorPosition(rotorPositionRotations);
            talonFxSimState.setRotorVelocity(rotorVelocityRotationsPerSecond);

            return talonFxSimState.getMotorVoltageMeasure();
        }
    }

    /**
     * MapleSim motor-controller adapter for a TalonFX + remote CANcoder.
     *
     * Use this for steer when your TalonFX uses remote CANcoder feedback.
     */
    public static final class TalonFXMotorControllerWithRemoteCancoderSim implements SimulatedMotorController {
        private final TalonFXSimState talonFxSimState;
        private final CANcoderSimState cancoderSimState;
        private final double rotorToMechanismRatio;

        public TalonFXMotorControllerWithRemoteCancoderSim(
            TalonFX talonFx,
            CANcoder cancoder,
            double rotorToMechanismRatio
        ) {
            this.talonFxSimState = talonFx.getSimState();
            this.cancoderSimState = cancoder.getSimState();
            this.rotorToMechanismRatio = rotorToMechanismRatio;
        }

        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            Voltage batteryVoltage = SimulatedBattery.getBatteryVoltage();
            talonFxSimState.setSupplyVoltage(batteryVoltage);
            cancoderSimState.setSupplyVoltage(batteryVoltage);

            double mechanismPositionRotations = mechanismAngle.in(Radians) / (2.0 * Math.PI);
            double mechanismVelocityRotationsPerSecond =
                mechanismVelocity.in(RadiansPerSecond) / (2.0 * Math.PI);

            double rotorPositionRotations = mechanismPositionRotations * rotorToMechanismRatio;
            double rotorVelocityRotationsPerSecond =
                mechanismVelocityRotationsPerSecond * rotorToMechanismRatio;

            talonFxSimState.setRawRotorPosition(rotorPositionRotations);
            talonFxSimState.setRotorVelocity(rotorVelocityRotationsPerSecond);

            double encoderPositionRotations = encoderAngle.in(Radians) / (2.0 * Math.PI);
            double encoderVelocityRotationsPerSecond =
                encoderVelocity.in(RadiansPerSecond) / (2.0 * Math.PI);

            cancoderSimState.setRawPosition(encoderPositionRotations);
            cancoderSimState.setVelocity(encoderVelocityRotationsPerSecond);

            return talonFxSimState.getMotorVoltageMeasure();
        }
    }
}