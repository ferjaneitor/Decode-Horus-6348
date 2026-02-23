package frc.robot.Climber.IO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ClimberConstants;

public final class ClimberIOSim implements ClimberIO {

    private static final double SIMULATION_TIME_STEP_SECONDS = 0.02;

    private static final double MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND = 25.0;
    private static final double VELOCITY_RESPONSE_TIME_CONSTANT_SECONDS = 0.15;

    private final PIDController leftPositionController;
    private final PIDController rightPositionController;

    private ClimberControlMode climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;

    private double climberTargetPositionRotations = 0.0;
    private double climberAppliedVoltageCommandVolts = 0.0;

    private double leftClimberPositionRotations = 0.0;
    private double leftClimberVelocityRotationsPerSecond = 0.0;

    private double rightClimberPositionRotations = 0.0;
    private double rightClimberVelocityRotationsPerSecond = 0.0;

    private double lastLeftFeedbackVolts = 0.0;
    private double lastLeftTotalCommandedVolts = 0.0;

    private double lastRightFeedbackVolts = 0.0;
    private double lastRightTotalCommandedVolts = 0.0;

    public ClimberIOSim() {
        leftPositionController = new PIDController(
                ClimberConstants.CLIMBER_MOTOR_CONFIG().kp,
                ClimberConstants.CLIMBER_MOTOR_CONFIG().ki,
                ClimberConstants.CLIMBER_MOTOR_CONFIG().kd
        );

        rightPositionController = new PIDController(
                ClimberConstants.CLIMBER_MOTOR_CONFIG().kp,
                ClimberConstants.CLIMBER_MOTOR_CONFIG().ki,
                ClimberConstants.CLIMBER_MOTOR_CONFIG().kd
        );

        leftPositionController.setIntegratorRange(-12.0, 12.0);
        rightPositionController.setIntegratorRange(-12.0, 12.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs climberInputs) {
        double batteryBusVoltageVolts = RobotController.getBatteryVoltage();

        double minimumPositionRotations = 0.0;
        double maximumPositionRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION;

        double clampedTargetRotations =
                MathUtil.clamp(climberTargetPositionRotations, minimumPositionRotations, maximumPositionRotations);

        double leftCommandedVoltageVolts;
        double rightCommandedVoltageVolts;

        if (climberControlMode == ClimberControlMode.POSITION_PID) {
            lastLeftFeedbackVolts =
                    leftPositionController.calculate(leftClimberPositionRotations, clampedTargetRotations);

            lastRightFeedbackVolts =
                    rightPositionController.calculate(rightClimberPositionRotations, clampedTargetRotations);

            leftCommandedVoltageVolts =
                    MathUtil.clamp(lastLeftFeedbackVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            rightCommandedVoltageVolts =
                    MathUtil.clamp(lastRightFeedbackVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            lastLeftTotalCommandedVolts = leftCommandedVoltageVolts;
            lastRightTotalCommandedVolts = rightCommandedVoltageVolts;
        } else {
            leftPositionController.reset();
            rightPositionController.reset();

            lastLeftFeedbackVolts = 0.0;
            lastRightFeedbackVolts = 0.0;

            leftCommandedVoltageVolts =
                    MathUtil.clamp(climberAppliedVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            rightCommandedVoltageVolts =
                    MathUtil.clamp(climberAppliedVoltageCommandVolts, -batteryBusVoltageVolts, batteryBusVoltageVolts);

            lastLeftTotalCommandedVolts = leftCommandedVoltageVolts;
            lastRightTotalCommandedVolts = rightCommandedVoltageVolts;
        }

        updateSideSimulation(leftCommandedVoltageVolts, batteryBusVoltageVolts, true);
        updateSideSimulation(rightCommandedVoltageVolts, batteryBusVoltageVolts, false);

        climberInputs.leftClimberConnected = true;
        climberInputs.leftClimberPositionRotations = leftClimberPositionRotations;
        climberInputs.leftClimberVelocityRotationsPerSecond = leftClimberVelocityRotationsPerSecond;
        climberInputs.leftClimberAppliedVolts = leftCommandedVoltageVolts;
        climberInputs.leftClimberCurrentAmps = estimateCurrentAmps(leftCommandedVoltageVolts, batteryBusVoltageVolts);

        climberInputs.rightClimberConnected = true;
        climberInputs.rightClimberPositionRotations = rightClimberPositionRotations;
        climberInputs.rightClimberVelocityRotationsPerSecond = rightClimberVelocityRotationsPerSecond;
        climberInputs.rightClimberAppliedVolts = rightCommandedVoltageVolts;
        climberInputs.rightClimberCurrentAmps = estimateCurrentAmps(rightCommandedVoltageVolts, batteryBusVoltageVolts);

        climberInputs.climberControlMode = climberControlMode;
        climberInputs.climberTargetPositionRotations = clampedTargetRotations;
        climberInputs.climberAppliedVoltageCommandVolts = climberAppliedVoltageCommandVolts;

        climberInputs.leftFeedbackVolts = lastLeftFeedbackVolts;
        climberInputs.leftTotalCommandedVolts = lastLeftTotalCommandedVolts;

        climberInputs.rightFeedbackVolts = lastRightFeedbackVolts;
        climberInputs.rightTotalCommandedVolts = lastRightTotalCommandedVolts;
    }

    private void updateSideSimulation(double commandedVoltageVolts, double batteryBusVoltageVolts, boolean isLeftSide) {
        double commandedPercentOutput =
                (Math.abs(batteryBusVoltageVolts) < 1e-6) ? 0.0 : (commandedVoltageVolts / batteryBusVoltageVolts);

        commandedPercentOutput = MathUtil.clamp(commandedPercentOutput, -1.0, 1.0);

        double desiredVelocityRotationsPerSecond =
                commandedPercentOutput * MAXIMUM_VELOCITY_ROTATIONS_PER_SECOND;

        if (isLeftSide) {
            leftClimberVelocityRotationsPerSecond +=
                    (desiredVelocityRotationsPerSecond - leftClimberVelocityRotationsPerSecond)
                            * (SIMULATION_TIME_STEP_SECONDS / VELOCITY_RESPONSE_TIME_CONSTANT_SECONDS);

            leftClimberPositionRotations += leftClimberVelocityRotationsPerSecond * SIMULATION_TIME_STEP_SECONDS;
            leftClimberPositionRotations = clampPosition(leftClimberPositionRotations);
        } else {
            rightClimberVelocityRotationsPerSecond +=
                    (desiredVelocityRotationsPerSecond - rightClimberVelocityRotationsPerSecond)
                            * (SIMULATION_TIME_STEP_SECONDS / VELOCITY_RESPONSE_TIME_CONSTANT_SECONDS);

            rightClimberPositionRotations += rightClimberVelocityRotationsPerSecond * SIMULATION_TIME_STEP_SECONDS;
            rightClimberPositionRotations = clampPosition(rightClimberPositionRotations);
        }
    }

    private double clampPosition(double positionRotations) {
        double minimumPositionRotations = 0.0;
        double maximumPositionRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION;
        return MathUtil.clamp(positionRotations, minimumPositionRotations, maximumPositionRotations);
    }

    private double estimateCurrentAmps(double commandedVoltageVolts, double batteryBusVoltageVolts) {
        if (Math.abs(batteryBusVoltageVolts) < 1e-6) {
            return 0.0;
        }
        double commandedPercentOutput = Math.abs(commandedVoltageVolts / batteryBusVoltageVolts);
        return 8.0 + 20.0 * commandedPercentOutput;
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;
        climberAppliedVoltageCommandVolts = voltage;
    }

    @Override
    public void stopClimber() {
        climberControlMode = ClimberControlMode.OPEN_LOOP_VOLTAGE;
        climberAppliedVoltageCommandVolts = 0.0;
    }

    @Override
    public void setClimberPositionPidRotations(double targetPositionRotations) {
        climberControlMode = ClimberControlMode.POSITION_PID;
        climberTargetPositionRotations = targetPositionRotations;
    }

    @Override
    public void setClimberRawEncoderPositionRotations(double leftPositionRotations, double rightPositionRotations) {
        leftClimberPositionRotations = leftPositionRotations;
        rightClimberPositionRotations = rightPositionRotations;
        leftClimberVelocityRotationsPerSecond = 0.0;
        rightClimberVelocityRotationsPerSecond = 0.0;
    }
}