package frc.SuperSubsystem.SuperMotors.SparkMax;

import com.revrobotics.spark.SparkBase;
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

    private final PIDController positionPidController;

    // Telemetry (MA-style)
    private double lastFeedforwardVolts = 0.0;
    private double lastFeedbackVolts = 0.0;
    private double lastTotalCommandedVolts = 0.0;

    public SuperSparkMax(int deviceCanIdentifier) {
        this(deviceCanIdentifier, DEFAULT_MOTOR_TYPE, new SuperSparkMaxConfig());
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType) {
        this(deviceCanIdentifier, motorType, new SuperSparkMaxConfig());
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType, SuperSparkMaxConfig userConfiguration) {
        this.userConfiguration = userConfiguration;

        SparkMaxSupport.SparkMaxConfigFactory configFactory = new SparkMaxSupport.SparkMaxConfigFactory();
        this.hardware = new SparkMaxSupport.SparkMaxHardware(
                deviceCanIdentifier,
                motorType,
                configFactory.createFrom(userConfiguration)
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

        this.positionPidController = new PIDController(userConfiguration.kp, userConfiguration.ki, userConfiguration.kd);
    }

    // ---------- Hardware passthrough ----------
    public SparkBase getSparkBase() {
        return hardware.getSparkBase();
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
        lastFeedforwardVolts = 0.0;
        lastFeedbackVolts = 0.0;
        lastTotalCommandedVolts = 0.0;
    }

    // ---------- Motion profile status ----------
    public void cancelProfile() {
        stop();
        motionProfileRunner.cancel();
    }

    public boolean isProfileActive() {
        return motionProfileRunner.isActive();
    }

    public boolean isProfileFinished() {
        return !motionProfileRunner.isActive();
    }

    public double getProfileElapsedTimeSeconds() {
        return motionProfileRunner.getProfileElapsedTimeSeconds();
    }

    public double getProfileTotalTimeSeconds() {
        return motionProfileRunner.getProfileTotalTimeSeconds();
    }

    // ---------- Telemetry getters ----------
    public SparkMaxMotionState getCurrentDesiredState() {
        return lastDesiredState;
    }

    public double getLastFeedforwardVolts() {
        return lastFeedforwardVolts;
    }

    public double getLastFeedbackVolts() {
        return lastFeedbackVolts;
    }

    public double getLastTotalCommandedVolts() {
        return lastTotalCommandedVolts;
    }

    // ---------- Simple PID position hold ----------
    public void PIDPositionControl(double desiredPositionRotations) {
        double currentPositionRotations = getRelativeEncoderPosition();
        double feedbackVoltage = positionPidController.calculate(currentPositionRotations, desiredPositionRotations);

        lastDesiredState = new SparkMaxMotionState(desiredPositionRotations, 0.0, 0.0, 0.0);
        lastFeedforwardVolts = 0.0;
        lastFeedbackVolts = feedbackVoltage;
        lastTotalCommandedVolts = MathUtil.clamp(feedbackVoltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);

        setVoltage(lastTotalCommandedVolts);
    }

    // ---------- Motion magic position ----------
    public void magicMotionPositionControl(double desiredPositionRotations) {
        magicMotionPositionControl(desiredPositionRotations, false);
    }

    public void magicMotionPositionControl(double desiredPositionRotations, boolean enableJerkControl) {
        MotionProfiles.MotionProfileType motionProfileType =
                enableJerkControl ? MotionProfiles.MotionProfileType.S_CURVE : MotionProfiles.MotionProfileType.TRAPEZOIDAL;

        double currentPositionRotations = getRelativeEncoderPosition();

        ensureProfileStartedIfNeeded(
                currentPositionRotations,
                desiredPositionRotations,
                motionProfileType,
                MotionProfiles.ProfileDomain.POSITION
        );

        SparkMaxMotionState desiredState = motionProfileRunner.sampleNowOrZero();
        lastDesiredState = desiredState;

        double feedforwardVoltage = feedforwardModel.calculateVoltage(
                currentPositionRotations,
                desiredState.velocity,
                desiredState.acceleration
        );

        double feedbackVoltage = feedbackController.calculatePositionFeedback(currentPositionRotations, desiredState.position);

        lastFeedforwardVolts = feedforwardVoltage;
        lastFeedbackVolts = feedbackVoltage;
        lastTotalCommandedVolts = MathUtil.clamp(feedforwardVoltage + feedbackVoltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);

        setVoltage(lastTotalCommandedVolts);
    }

    // ---------- Motion magic velocity ----------
    public void magicMotionVelocityControl(double desiredVelocityRotationsPerSecond) {
        magicMotionVelocityControl(desiredVelocityRotationsPerSecond, false);
    }

    public void magicMotionVelocityControl(double desiredVelocityRotationsPerSecond, boolean enableJerkControl) {
        MotionProfiles.MotionProfileType motionProfileType =
                enableJerkControl ? MotionProfiles.MotionProfileType.S_CURVE : MotionProfiles.MotionProfileType.TRAPEZOIDAL;

        double currentVelocityRotationsPerSecond = getRelativeEncoderVelocity();

        ensureProfileStartedIfNeeded(
                currentVelocityRotationsPerSecond,
                desiredVelocityRotationsPerSecond,
                motionProfileType,
                MotionProfiles.ProfileDomain.VELOCITY
        );

        SparkMaxMotionState desiredState = motionProfileRunner.sampleNowOrZero();
        lastDesiredState = desiredState;

        double currentPositionRotations = getRelativeEncoderPosition();

        double feedforwardVoltage = feedforwardModel.calculateVoltage(
                currentPositionRotations,
                desiredState.velocity,
                desiredState.acceleration
        );

        double feedbackVoltage = feedbackController.calculateVelocityFeedback(currentVelocityRotationsPerSecond, desiredState.velocity);

        lastFeedforwardVolts = feedforwardVoltage;
        lastFeedbackVolts = feedbackVoltage;
        lastTotalCommandedVolts = MathUtil.clamp(feedforwardVoltage + feedbackVoltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);

        setVoltage(lastTotalCommandedVolts);
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

    public void positionControlVoltage(double desiredPositionRotations) {
        PIDPositionControl(desiredPositionRotations);
    }

    public void setRelativeEncoderPositionRotations(double desiredPositionRotations) {
        getSparkBase().getEncoder().setPosition(desiredPositionRotations);

        positionPidController.reset();
        feedbackController.reset();
        motionProfileRunner.cancel();

        lastFeedforwardVolts = 0.0;
        lastFeedbackVolts = 0.0;
        lastTotalCommandedVolts = 0.0;
    }

}
