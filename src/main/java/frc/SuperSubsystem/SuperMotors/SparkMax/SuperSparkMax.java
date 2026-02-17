package frc.SuperSubsystem.SuperMotors.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SparkMaxMotionState;
import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SuperSparkMaxConfig;
import frc.robot.Constants;

public final class SuperSparkMax {
    private static final double MAXIMUM_BATTERY_VOLTAGE = Constants.BATTERY_VOLTAGE;
    private static final MotorType DEFAULT_MOTOR_TYPE = MotorType.kBrushless;

    private final SparkMaxSupport.SparkMaxHardware hardware;
    private final SparkMaxSupport.FeedforwardModel feedforwardModel;
    private final SparkMaxSupport.FeedbackController feedbackController;
    private final SparkMaxSupport.MotionProfileRunner motionProfileRunner;

    private final SuperSparkMaxConfig userConfiguration;

    private SparkMaxMotionState lastDesiredState;
    private MotionProfiles.MotionProfileType lastMotionProfileType;

    private final PIDController pidController;

    public SuperSparkMax(int deviceCanIdentifier) {
        this(deviceCanIdentifier, DEFAULT_MOTOR_TYPE, new SuperSparkMaxConfig());
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType) {
        this(deviceCanIdentifier, motorType, new SuperSparkMaxConfig());
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType, SuperSparkMaxConfig userConfiguration) {
        this.userConfiguration = userConfiguration;

        SparkMaxSupport.SparkMaxConfigFactory sparkMaxConfigFactory = new SparkMaxSupport.SparkMaxConfigFactory();
        this.hardware = new SparkMaxSupport.SparkMaxHardware(
                deviceCanIdentifier,
                motorType,
                sparkMaxConfigFactory.createFrom(userConfiguration)
        );

        this.feedforwardModel = new SparkMaxSupport.FeedforwardModel(userConfiguration);
        this.feedbackController = new SparkMaxSupport.FeedbackController(
                userConfiguration.kp,
                userConfiguration.ki,
                userConfiguration.kd,
                MAXIMUM_BATTERY_VOLTAGE
        );

        this.motionProfileRunner = new SparkMaxSupport.MotionProfileRunner(new SparkMaxSupport.SystemTimeProvider());

        this.lastDesiredState = new SparkMaxMotionState(0, 0, 0, 0);
        this.lastMotionProfileType = MotionProfiles.MotionProfileType.TRAPEZOIDAL;

        this.pidController = new PIDController(userConfiguration.kp, userConfiguration.ki, userConfiguration.kd);
    }

    public double getRelativeEncoderPosition() {
        return hardware.getPosition();
    }

    public double getRelativeEncoderVelocity() {
        return hardware.getVelocity();
    }

    public void setDutyCycle(double dutyCycle) {
        hardware.setDutyCycle(dutyCycle);
    }

    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
        hardware.setVoltage(clampedVoltage);
    }

    public void stop() {
        hardware.stop();
    }

    public void cancelProfile() {
        stop();
        motionProfileRunner.cancel();
    }

    public boolean isProfileFinished() {
        return !motionProfileRunner.isActive();
    }

    public SparkMaxMotionState getCurrentDesiredState() {
        return lastDesiredState;
    }

    public void PIDPositionControl(double desiredPosition){
        double currentPosition = getRelativeEncoderPosition();
        double output = pidController.calculate(currentPosition, desiredPosition);
        setVoltage(output);
    }

    public void magicMotionPositionControl(double desiredPosition) {
        magicMotionPositionControl(desiredPosition, false);
    }

    public void magicMotionPositionControl(double desiredPosition, boolean enableJerkControl) {
        MotionProfiles.MotionProfileType motionProfileType =
                enableJerkControl ? MotionProfiles.MotionProfileType.S_CURVE : MotionProfiles.MotionProfileType.TRAPEZOIDAL;

        double currentPosition = getRelativeEncoderPosition();
        ensureProfileStartedIfNeeded(currentPosition, desiredPosition, motionProfileType, MotionProfiles.ProfileDomain.POSITION);

        SparkMaxMotionState desiredState = motionProfileRunner.sampleNowOrZero();
        lastDesiredState = desiredState;

        double feedforwardVoltage = feedforwardModel.calculateVoltage(
                currentPosition,
                desiredState.velocity,
                desiredState.acceleration
        );

        double feedbackVoltage = feedbackController.calculatePositionFeedback(currentPosition, desiredState.position);

        setVoltage(feedforwardVoltage + feedbackVoltage);
    }

    public void magicMotionVelocityControl(double desiredVelocity) {
        magicMotionVelocityControl(desiredVelocity, false);
    }

    public void magicMotionVelocityControl(double desiredVelocity, boolean enableJerkControl) {
        MotionProfiles.MotionProfileType motionProfileType =
                enableJerkControl ? MotionProfiles.MotionProfileType.S_CURVE : MotionProfiles.MotionProfileType.TRAPEZOIDAL;

        double currentVelocity = getRelativeEncoderVelocity();
        ensureProfileStartedIfNeeded(currentVelocity, desiredVelocity, motionProfileType, MotionProfiles.ProfileDomain.VELOCITY);

        SparkMaxMotionState desiredState = motionProfileRunner.sampleNowOrZero();
        lastDesiredState = desiredState;

        double currentPosition = getRelativeEncoderPosition();

        double feedforwardVoltage = feedforwardModel.calculateVoltage(
                currentPosition,
                desiredState.velocity,
                desiredState.acceleration
        );

        double feedbackVoltage = feedbackController.calculateVelocityFeedback(currentVelocity, desiredState.velocity);

        setVoltage(feedforwardVoltage + feedbackVoltage);
    }

    private void ensureProfileStartedIfNeeded(
            double startValue,
            double targetValue,
            MotionProfiles.MotionProfileType motionProfileType,
            MotionProfiles.ProfileDomain profileDomain
    ) {
        double targetChangeTolerance = 0.001;
        boolean profileTypeChanged = (motionProfileType != lastMotionProfileType);

        if (!motionProfileRunner.isActive()
                || motionProfileRunner.targetChanged(targetValue, targetChangeTolerance)
                || profileTypeChanged
                || motionProfileRunner.domainChanged(profileDomain)) {

            feedbackController.reset();

            MotionProfiles.MotionProfile motionProfile = createMotionProfile(startValue, targetValue, motionProfileType);
            motionProfileRunner.start(motionProfile, targetValue, profileDomain);

            lastMotionProfileType = motionProfileType;
        }
    }

    private MotionProfiles.MotionProfile createMotionProfile(
            double startValue,
            double targetValue,
            MotionProfiles.MotionProfileType motionProfileType
    ) {
        if (motionProfileType == MotionProfiles.MotionProfileType.S_CURVE) {
            return new MotionProfiles.SCurveMotionProfile(
                    startValue,
                    targetValue,
                    userConfiguration.CruiseVelocity,
                    userConfiguration.TargetAcceleration,
                    userConfiguration.TargetJerk
            );
        }

        return new MotionProfiles.TrapezoidalMotionProfile(
                startValue,
                targetValue,
                userConfiguration.CruiseVelocity,
                userConfiguration.TargetAcceleration
        );
    }
}
