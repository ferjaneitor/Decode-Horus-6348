package frc.SuperSubsystem.SuperMotors.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SparkMaxMotionState;
import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SuperSparkMaxConfig;

public class SuperSparkMax {

    private static final double MAXIMUM_BATTERY_VOLTAGE = 12.0;

    private static final MotorType DEFAULT_MOTOR_TYPE = MotorType.kBrushless;

    private static final SuperSparkMaxConfig DEFAULT_CONFIG = new SuperSparkMaxConfig();

    private final int deviceCanIdentifier;
    private final MotorType motorType;

    private final SparkMax sparkMaxMotorController;
    private final RelativeEncoder relativeEncoder;

    private final SuperSparkMaxConfig userConfiguration;
    private final SparkMaxConfig revConfiguration;

    private final PIDController pidControllerVoltage, pidControllerDutyCycle;

    private final SparkMaxMotionState motionState = new SparkMaxMotionState(0, 0,0,0);

    public SuperSparkMax(int deviceCanIdentifier) {
        this(deviceCanIdentifier, DEFAULT_MOTOR_TYPE, DEFAULT_CONFIG);
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType) {
        this(deviceCanIdentifier, motorType, DEFAULT_CONFIG);
    }

    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType, SuperSparkMaxConfig userConfiguration) {
        this.deviceCanIdentifier = deviceCanIdentifier;
        this.motorType = motorType;

        this.sparkMaxMotorController = new SparkMax(this.deviceCanIdentifier, this.motorType);

        // Default relative encoder (NEO hall sensor on brushless)
        this.relativeEncoder = this.sparkMaxMotorController.getEncoder();

        this.userConfiguration = userConfiguration;
        this.revConfiguration = buildRevConfigurationFromUser(userConfiguration);

        this.sparkMaxMotorController.configure(
                this.revConfiguration,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
        );

        // If you want, log configureResult when not OK.

        this.pidControllerVoltage = new PIDController(
                this.userConfiguration.kp,
                this.userConfiguration.ki,
                this.userConfiguration.kd
        );

        this.pidControllerDutyCycle = new PIDController(
                this.userConfiguration.kp,
                this.userConfiguration.ki,
                this.userConfiguration.kd
        );

        this.pidControllerVoltage.setIntegratorRange(-MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE); 
        this.pidControllerDutyCycle.setIntegratorRange(-1.0, 1.0); 
    }

    private SparkMaxConfig buildRevConfigurationFromUser(SuperSparkMaxConfig userConfiguration) {
        SparkMaxConfig configuration = new SparkMaxConfig();

        configuration.idleMode(userConfiguration.kIsBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        configuration.inverted(userConfiguration.kIsInverted);

        configuration.smartCurrentLimit(
                userConfiguration.SmartCurrentStallLimit,
                userConfiguration.SmartCurrentFreeLimit
        );

        configuration.openLoopRampRate(userConfiguration.kRampRate);

        configuration.softLimit
                .forwardSoftLimit(userConfiguration.kSoftLimitFwd)
                .forwardSoftLimitEnabled(userConfiguration.kSoftLimitFwdEnabled)
                .reverseSoftLimit(userConfiguration.kSoftLimitRev)
                .reverseSoftLimitEnabled(userConfiguration.kSoftLimitRevEnabled);

        configuration.encoder
                .inverted(userConfiguration.kIsEncoderInverted)
                .positionConversionFactor(userConfiguration.kPositionConvertionFactor)
                .velocityConversionFactor(userConfiguration.kVelocityConversionFactor);

        return configuration;
    }

    public double getRelativeEncoderPosition() {
        return this.relativeEncoder.getPosition();
    }

    public double getRelativeEncoderVelocity() {
        return this.relativeEncoder.getVelocity();
    }

    public void setDutyCycle(double dutyCycle) {
        this.sparkMaxMotorController.set(MathUtil.clamp(dutyCycle, -1.0, 1.0));
    }

    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
        this.sparkMaxMotorController.setVoltage(clampedVoltage);
    }

    public void resetPid() {
        this.pidControllerVoltage.reset();
        this.pidControllerDutyCycle.reset();
    }

    // PID outputs volts directly
    public void velocityControlVoltage(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredVelocity, desiredVelocity);
        this.setVoltage(pidVoltageOutput);
    }

    // PID outputs volts directly
    public void positionControlVoltage(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredPosition, desiredPosition);
        this.setVoltage(pidVoltageOutput);
    }

    // PID outputs volts directly
    public void velocityControlDutyCycle(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidDutyCycleOutput = this.pidControllerDutyCycle.calculate(measuredVelocity, desiredVelocity);
        this.setDutyCycle(pidDutyCycleOutput);
    }

    // PID outputs volts directly
    public void positionControlDutyCycle(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidDutyCycleOutput = this.pidControllerDutyCycle.calculate(measuredPosition, desiredPosition);
        this.setDutyCycle(pidDutyCycleOutput);
    }

    public void stop() {
        this.sparkMaxMotorController.stopMotor();
    }

    public double calculateFeedForward(double desiredVelocity, double desiredAcceleration) {
        this.userConfiguration.kGcos(this.getRelativeEncoderPosition()); // Update kGcos based on current position
        double feedforward =0.0;
        feedforward += this.userConfiguration.Kg * Math.cos(this.userConfiguration.kGcos) * this.userConfiguration.kGcosRatio; // Assuming kGcos is the cosine of the angle, and kGcosRatio is a scaling factor
        feedforward +=this.userConfiguration.Ka * desiredAcceleration;
        feedforward += this.userConfiguration.Kv * desiredVelocity;
        feedforward += this.userConfiguration.Ks * Math.signum(desiredVelocity);

        return feedforward;
    }

    public double PIDFController(double desiredVelocity, double desiredAcceleration) {
        double feedforward = this.calculateFeedForward(desiredVelocity, desiredAcceleration);
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredVelocity, desiredVelocity);
        double output = feedforward + pidVoltageOutput;
        return MathUtil.clamp(output, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
    }

    public double CalculateProfileMotion(double setPoint, double currentSetpoint, boolean SprofileOrTrapezoidal, boolean isPositionControl){
        return 0;
    }

    public void MagicMotionPositionControl(double desiredPosition) {
    }

    public void MagicMotionVelocityControl(double desiredVelocity) {
    }

    public void MagicMotionVelocityControl(double desiredVelocity, boolean enableJerk){
    }

    private double stepForPosition ( SparkMaxMotionState motionState, boolean enableJerk, boolean isPositiveDirection) {
        
        if (userConfiguration.dt <= 0) {
            return 0;
        }

        if (enableJerk) {
            
            if  (userConfiguration.TargetJerk > 0) {
                double jerk = userConfiguration.TargetJerk * (isPositiveDirection ? 1 : -1);
                double acceleration = motionState.acceleration + jerk * userConfiguration.dt;
                acceleration = MathUtil.clamp(acceleration, -userConfiguration.TargetAcceleration, userConfiguration.TargetAcceleration);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                velocity = MathUtil.clamp(velocity, -userConfiguration.CruiseVelocity, userConfiguration.CruiseVelocity);
                double position = motionState.position + velocity * userConfiguration.dt;
                return position;
            }
             else {
                return 0;
             }

        } else {

            if ( userConfiguration.TargetAcceleration > 0) {
                double acceleration = userConfiguration.TargetAcceleration * (isPositiveDirection ? 1 : -1);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                velocity = MathUtil.clamp(velocity, -userConfiguration.CruiseVelocity, userConfiguration.CruiseVelocity);
                double position = motionState.position + velocity * userConfiguration.dt;
                return position;
            }
             else {
                return 0;
             }
            
        }
        
    }

    private double stepForVelocity ( SparkMaxMotionState motionState, boolean enableJerk, boolean isPositiveDirection) {
        if (userConfiguration.dt <= 0) {
            return 0;
        }

        if (enableJerk) {
            
            if  (userConfiguration.TargetJerk > 0) {
                double jerk = userConfiguration.TargetJerk * (isPositiveDirection ? 1 : -1);
                double acceleration = motionState.acceleration + jerk * userConfiguration.dt;
                acceleration = MathUtil.clamp(acceleration, -userConfiguration.TargetAcceleration, userConfiguration.TargetAcceleration);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                return velocity;
            }
             else {
                return 0;
             }

        } else {

            if ( userConfiguration.TargetAcceleration > 0) {
                double acceleration = userConfiguration.TargetAcceleration * (isPositiveDirection ? 1 : -1);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                return velocity;
            }
             else {
                return 0;
             }
            
        }
        
    }
}
