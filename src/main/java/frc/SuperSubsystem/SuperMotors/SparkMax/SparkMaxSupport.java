package frc.SuperSubsystem.SuperMotors.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SuperSparkMaxConfig;
import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SparkMaxMotionState;

public final class SparkMaxSupport {
    private SparkMaxSupport() {}

    public interface TimeProvider {
        double nowSeconds();
    }

    public static final class SystemTimeProvider implements TimeProvider {
        @Override
        public double nowSeconds() {
            return System.currentTimeMillis() / 1000.0;
        }
    }

    public static final class SparkMaxConfigFactory {
        public SparkMaxConfig createFrom(SuperSparkMaxConfig userConfiguration) {
            SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

            sparkMaxConfig.idleMode(userConfiguration.kIsBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            sparkMaxConfig.inverted(userConfiguration.kIsInverted);

            sparkMaxConfig.smartCurrentLimit(
                    userConfiguration.SmartCurrentStallLimit,
                    userConfiguration.SmartCurrentFreeLimit
            );

            sparkMaxConfig.openLoopRampRate(userConfiguration.kRampRate);

            sparkMaxConfig.softLimit
                    .forwardSoftLimit(userConfiguration.kSoftLimitFwd)
                    .forwardSoftLimitEnabled(userConfiguration.kSoftLimitFwdEnabled)
                    .reverseSoftLimit(userConfiguration.kSoftLimitRev)
                    .reverseSoftLimitEnabled(userConfiguration.kSoftLimitRevEnabled);

            sparkMaxConfig.encoder
                    .inverted(userConfiguration.kIsEncoderInverted)
                    .positionConversionFactor(userConfiguration.kPositionConvertionFactor)
                    .velocityConversionFactor(userConfiguration.kVelocityConversionFactor);

            return sparkMaxConfig;
        }
    }

    public static final class SparkMaxHardware {
        private final SparkMax sparkMaxController;
        private final RelativeEncoder relativeEncoder;

        public SparkMaxHardware(int deviceCanIdentifier, MotorType motorType, SparkMaxConfig sparkMaxConfig) {
            this.sparkMaxController = new SparkMax(deviceCanIdentifier, motorType);
            this.relativeEncoder = this.sparkMaxController.getEncoder();

            this.sparkMaxController.configure(
                    sparkMaxConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters
            );
        }

        public double getPosition() {
            return relativeEncoder.getPosition();
        }

        public double getVelocity() {
            return relativeEncoder.getVelocity();
        }

        public void setDutyCycle(double dutyCycle) {
            sparkMaxController.set(MathUtil.clamp(dutyCycle, -1.0, 1.0));
        }

        public void setVoltage(double voltage) {
            sparkMaxController.setVoltage(voltage);
        }

        public void stop() {
            sparkMaxController.stopMotor();
        }
        public SparkBase getSparkBase() {
            return sparkMaxController;
        }

        public double getAppliedOutput() {
            return sparkMaxController.getAppliedOutput();
        }

        public double getBusVoltage() {
            return sparkMaxController.getBusVoltage();
        }

        public double getOutputCurrent() {
            return sparkMaxController.getOutputCurrent();
        }

        public void setEncoderPosition(double position) {
            relativeEncoder.setPosition(position);
        }
    }

    public static final class FeedforwardModel {
        private final SuperSparkMaxConfig userConfiguration;

        public FeedforwardModel(SuperSparkMaxConfig userConfiguration) {
            this.userConfiguration = userConfiguration;
        }

        public double calculateVoltage(double currentPosition, double desiredVelocity, double desiredAcceleration) {
            userConfiguration.kGcos(currentPosition);

            double gravityVoltage =
                    userConfiguration.Kg *
                    Math.cos(userConfiguration.kGcos) *
                    userConfiguration.kGcosRatio;

            double accelerationVoltage = userConfiguration.Ka * desiredAcceleration;
            double velocityVoltage = userConfiguration.Kv * desiredVelocity;

            double staticFrictionVoltage = 0.0;
            if (Math.abs(desiredVelocity) > 0.001) {
                staticFrictionVoltage = userConfiguration.Ks * Math.signum(desiredVelocity);
            }

            return gravityVoltage + accelerationVoltage + velocityVoltage + staticFrictionVoltage;
        }
    }

    public static final class FeedbackController {
        private final PIDController positionPidController;
        private final PIDController velocityPidController;

        public FeedbackController(double proportionalGain, double integralGain, double derivativeGain, double maximumOutput) {
            this.positionPidController = new PIDController(proportionalGain, integralGain, derivativeGain);
            this.velocityPidController = new PIDController(proportionalGain, integralGain, derivativeGain);

            this.positionPidController.setIntegratorRange(-maximumOutput, maximumOutput);
            this.velocityPidController.setIntegratorRange(-maximumOutput, maximumOutput);
        }

        public void reset() {
            positionPidController.reset();
            velocityPidController.reset();
        }

        public double calculatePositionFeedback(double measuredPosition, double desiredPosition) {
            return positionPidController.calculate(measuredPosition, desiredPosition);
        }

        public double calculateVelocityFeedback(double measuredVelocity, double desiredVelocity) {
            return velocityPidController.calculate(measuredVelocity, desiredVelocity);
        }
    }

    public static final class MotionProfileRunner {
        private final TimeProvider timeProvider;

        private MotionProfiles.MotionProfile activeMotionProfile;
        private double profileStartTimeSeconds;
        private double lastTargetValue;
        private boolean profileActive;

        private MotionProfiles.ProfileDomain activeDomain;

        public MotionProfileRunner(TimeProvider timeProvider) {
            this.timeProvider = timeProvider;
            this.profileActive = false;
            this.lastTargetValue = Double.NaN;
            this.activeDomain = MotionProfiles.ProfileDomain.POSITION;
        }

        public boolean isActive() {
            return profileActive;
        }

        public void cancel() {
            profileActive = false;
        }

        public boolean targetChanged(double newTargetValue, double targetChangeTolerance) {
            if (Double.isNaN(lastTargetValue)) {
                return true;
            }
            return Math.abs(lastTargetValue - newTargetValue) > targetChangeTolerance;
        }

        public boolean domainChanged(MotionProfiles.ProfileDomain newDomain) {
            return activeDomain != newDomain;
        }

        public void start(MotionProfiles.MotionProfile motionProfile, double targetValue, MotionProfiles.ProfileDomain domain) {
            this.activeMotionProfile = motionProfile;
            this.profileStartTimeSeconds = timeProvider.nowSeconds();
            this.lastTargetValue = targetValue;
            this.profileActive = true;
            this.activeDomain = domain;
        }

        public SparkMaxMotionState sampleNowOrZero() {
            if (!profileActive || activeMotionProfile == null) {
                return new SparkMaxMotionState(0, 0, 0, 0);
            }

            double elapsedTimeSeconds = timeProvider.nowSeconds() - profileStartTimeSeconds;
            SparkMaxMotionState desiredState = activeMotionProfile.sample(elapsedTimeSeconds);

            if (elapsedTimeSeconds >= activeMotionProfile.getTotalTimeSeconds()) {
                profileActive = false;
            }

            return desiredState;
        }

        public double getProfileElapsedTimeSeconds() { return timeProvider.nowSeconds() - profileStartTimeSeconds; }
        public double getProfileTotalTimeSeconds() { return activeMotionProfile != null ? activeMotionProfile.getTotalTimeSeconds() : 0.0; }

        
    }
    
}
