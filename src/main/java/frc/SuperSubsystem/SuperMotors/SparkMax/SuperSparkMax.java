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

/**
 * Enhanced SparkMax motor controller wrapper with advanced motion profiling capabilities.
 * 
 * This class provides:
 * - Trapezoidal motion profiles (3 phases: acceleration, cruise, deceleration)
 * - S-Curve motion profiles (7 phases: smooth jerk-controlled motion)
 * - Feedforward control (static friction, velocity, acceleration, gravity compensation)
 * - Proportional-Integral-Derivative (PID) feedback control
 * - Combined feedforward + feedback for optimal trajectory tracking
 * 
 * Motion Profile Types:
 * - Trapezoidal: Faster but with instantaneous acceleration changes (infinite jerk)
 * - S-Curve: Smoother with controlled jerk, better for oscillation-prone mechanisms
 * 
 * @author FRC Team
 * @version 1.0
 */
public class SuperSparkMax {

    // ============ CONSTANTS ============
    
    /** Maximum battery voltage in competition (12 volts) */
    private static final double MAXIMUM_BATTERY_VOLTAGE = 12.0;
    
    /** Default motor type if not specified (brushless NEO) */
    private static final MotorType DEFAULT_MOTOR_TYPE = MotorType.kBrushless;
    
    /** Default configuration with all parameters set to safe defaults */
    private static final SuperSparkMaxConfig DEFAULT_CONFIGURATION = new SuperSparkMaxConfig();

    // ============ HARDWARE COMPONENTS ============
    
    /** CAN bus identifier for this motor controller */
    private final int deviceCanIdentifier;
    
    /** Type of motor (brushless or brushed) */
    private final MotorType motorType;

    /** The underlying REV SparkMax motor controller hardware */
    private final SparkMax sparkMaxMotorController;
    
    /** Integrated encoder from the motor (NEO hall sensor for brushless) */
    private final RelativeEncoder relativeEncoder;

    /** User-provided configuration containing feedforward, limits, and PID gains */
    private final SuperSparkMaxConfig userConfiguration;
    
    /** REV Robotics native configuration built from user configuration */
    private final SparkMaxConfig revRoboticsConfiguration;

    /** Proportional-Integral-Derivative controller operating on voltage output */
    private final PIDController proportionalIntegralDerivativeControllerVoltage;
    
    /** Proportional-Integral-Derivative controller operating on duty cycle output */
    private final PIDController proportionalIntegralDerivativeControllerDutyCycle;

    // ============ MOTION PROFILE STATE TRACKING ============
    
    /** Position where the current motion profile started (encoder units) */
    private double profileStartPosition;
    
    /** Target position where the motion profile should end (encoder units) */
    private double profileTargetPosition;
    
    /** System time when the motion profile was initialized (seconds) */
    private double profileStartTime;
    
    /** Whether a motion profile is currently executing */
    private boolean profileIsActive;
    
    /** Whether the current profile uses jerk control (S-Curve) or not (Trapezoidal) */
    private boolean profileUsesJerkControl;
    
    /** Current desired position according to the motion profile (encoder units) */
    private double currentProfilePosition;
    
    /** Current desired velocity according to the motion profile (encoder units/sec) */
    private double currentProfileVelocity;
    
    /** Current desired acceleration according to the motion profile (encoder units/sec²) */
    private double currentProfileAcceleration;
    
    /** Current desired jerk according to the motion profile (encoder units/sec³) */
    private double currentProfileJerk;
    
    // ============ TRAPEZOIDAL PROFILE TIMING ============
    
    /** Duration of acceleration phase in trapezoidal profile (seconds) */
    private double trapezoidalAccelerationTime;
    
    /** Duration of constant velocity cruise phase in trapezoidal profile (seconds) */
    private double trapezoidalCruiseTime;
    
    /** Duration of deceleration phase in trapezoidal profile (seconds) */
    private double trapezoidalDecelerationTime;
    
    /** Total duration of the trapezoidal motion profile (seconds) */
    private double trapezoidalTotalProfileTime;
    
    // ============ S-CURVE PROFILE TIMING (7 PHASES) ============
    
    /** Phase 1: Duration of positive jerk (acceleration increasing) (seconds) */
    private double sCurvePhase1Time;
    
    /** Phase 2: Duration of constant acceleration (seconds) */
    private double sCurvePhase2Time;
    
    /** Phase 3: Duration of negative jerk (acceleration decreasing to zero) (seconds) */
    private double sCurvePhase3Time;
    
    /** Phase 4: Duration of constant velocity cruise (seconds) */
    private double sCurvePhase4Time;
    
    /** Phase 5: Duration of negative jerk (deceleration increasing) (seconds) */
    private double sCurvePhase5Time;
    
    /** Phase 6: Duration of constant deceleration (seconds) */
    private double sCurvePhase6Time;
    
    /** Phase 7: Duration of positive jerk (deceleration decreasing to zero) (seconds) */
    private double sCurvePhase7Time;
    
    /** Total duration of the S-Curve motion profile (seconds) */
    private double sCurveTotalProfileTime;

    // ============ CONSTRUCTORS ============

    /**
     * Creates a SuperSparkMax with default motor type and configuration.
     * 
     * @param deviceCanIdentifier CAN ID of the SparkMax controller (1-62)
     */
    public SuperSparkMax(int deviceCanIdentifier) {
        this(deviceCanIdentifier, DEFAULT_MOTOR_TYPE, DEFAULT_CONFIGURATION);
    }

    /**
     * Creates a SuperSparkMax with specified motor type and default configuration.
     * 
     * @param deviceCanIdentifier CAN ID of the SparkMax controller (1-62)
     * @param motorType Type of motor (kBrushless for NEO, kBrushed for CIM/775)
     */
    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType) {
        this(deviceCanIdentifier, motorType, DEFAULT_CONFIGURATION);
    }

    /**
     * Creates a SuperSparkMax with full custom configuration.
     * 
     * This is the main constructor that initializes all hardware and software components.
     * 
     * Initialization steps:
     * 1. Store hardware identifiers
     * 2. Create SparkMax controller and encoder objects
     * 3. Convert user configuration to REV native format
     * 4. Apply configuration to hardware
     * 5. Create PID controllers with proper output limits
     * 6. Initialize motion profile state to inactive
     * 
     * @param deviceCanIdentifier CAN ID of the SparkMax controller (1-62)
     * @param motorType Type of motor connected
     * @param userConfiguration Configuration with feedforward gains, PID, limits, etc.
     */
    public SuperSparkMax(int deviceCanIdentifier, MotorType motorType, SuperSparkMaxConfig userConfiguration) {
        // Store identifiers
        this.deviceCanIdentifier = deviceCanIdentifier;
        this.motorType = motorType;

        // Initialize hardware components
        this.sparkMaxMotorController = new SparkMax(this.deviceCanIdentifier, this.motorType);
        this.relativeEncoder = this.sparkMaxMotorController.getEncoder();

        // Build and apply configuration
        this.userConfiguration = userConfiguration;
        this.revRoboticsConfiguration = buildRevRoboticsConfigurationFromUserConfiguration(userConfiguration);

        this.sparkMaxMotorController.configure(
                this.revRoboticsConfiguration,
                ResetMode.kNoResetSafeParameters,  // Don't reset parameters already on device
                PersistMode.kNoPersistParameters    // Don't save to flash (faster startup)
        );

        // Create PID controllers for feedback control
        this.proportionalIntegralDerivativeControllerVoltage = new PIDController(
                this.userConfiguration.kp,
                this.userConfiguration.ki,
                this.userConfiguration.kd
        );

        this.proportionalIntegralDerivativeControllerDutyCycle = new PIDController(
                this.userConfiguration.kp,
                this.userConfiguration.ki,
                this.userConfiguration.kd
        );

        // Set integral windup limits to prevent excessive integral accumulation
        this.proportionalIntegralDerivativeControllerVoltage.setIntegratorRange(
            -MAXIMUM_BATTERY_VOLTAGE, 
            MAXIMUM_BATTERY_VOLTAGE
        );
        this.proportionalIntegralDerivativeControllerDutyCycle.setIntegratorRange(-1.0, 1.0);
        
        // Initialize motion profile state to inactive
        this.profileIsActive = false;
        this.profileUsesJerkControl = false;
        this.profileStartPosition = 0;
        this.profileTargetPosition = 0;
        this.profileStartTime = 0;
        this.currentProfilePosition = 0;
        this.currentProfileVelocity = 0;
        this.currentProfileAcceleration = 0;
        this.currentProfileJerk = 0;
    }

    // ============ CONFIGURATION BUILDING ============

    /**
     * Converts user configuration to REV Robotics native SparkMax configuration format.
     * 
     * This method maps our custom configuration parameters to the REV API format.
     * Configuration includes:
     * - Idle mode (brake vs coast)
     * - Motor inversion
     * - Current limits (prevent breaker trips and motor burnout)
     * - Ramp rate (voltage change limiting)
     * - Soft limits (software-enforced position limits)
     * - Encoder settings (inversion and conversion factors)
     * 
     * @param userConfiguration Our custom configuration object
     * @return SparkMaxConfig object ready to apply to hardware
     */
    private SparkMaxConfig buildRevRoboticsConfigurationFromUserConfiguration(SuperSparkMaxConfig userConfiguration) {
        SparkMaxConfig configuration = new SparkMaxConfig();

        // Set idle mode: brake holds position, coast allows free spinning
        configuration.idleMode(userConfiguration.kIsBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        
        // Set motor direction: inverted reverses positive direction
        configuration.inverted(userConfiguration.kIsInverted);

        // Configure smart current limiting to protect motor and battery
        // Stall limit: maximum current when motor is stalled (not moving)
        // Free limit: maximum current when motor is spinning freely
        configuration.smartCurrentLimit(
                userConfiguration.SmartCurrentStallLimit,
                userConfiguration.SmartCurrentFreeLimit
        );

        // Set open loop ramp rate: limits how fast voltage can change
        // Prevents sudden jerks and current spikes
        configuration.openLoopRampRate(userConfiguration.kRampRate);

        // Configure software position limits to prevent mechanical damage
        configuration.softLimit
                .forwardSoftLimit(userConfiguration.kSoftLimitFwd)
                .forwardSoftLimitEnabled(userConfiguration.kSoftLimitFwdEnabled)
                .reverseSoftLimit(userConfiguration.kSoftLimitRev)
                .reverseSoftLimitEnabled(userConfiguration.kSoftLimitRevEnabled);

        // Configure encoder: conversion factors turn motor rotations into meaningful units
        // Position factor: rotations -> meters, radians, etc.
        // Velocity factor: RPM -> meters/sec, radians/sec, etc.
        configuration.encoder
                .inverted(userConfiguration.kIsEncoderInverted)
                .positionConversionFactor(userConfiguration.kPositionConvertionFactor)
                .velocityConversionFactor(userConfiguration.kVelocityConversionFactor);

        return configuration;
    }

    // ============ ENCODER READING METHODS ============
    
    /**
     * Gets the current position from the integrated encoder.
     * 
     * The position is in user-configured units based on the position conversion factor.
     * For example:
     * - Drivetrain: meters
     * - Arm: radians
     * - Elevator: meters
     * 
     * @return Current position in configured units
     */
    public double getRelativeEncoderPosition() {
        return this.relativeEncoder.getPosition();
    }

    /**
     * Gets the current velocity from the integrated encoder.
     * 
     * The velocity is in user-configured units per second based on velocity conversion factor.
     * For example:
     * - Drivetrain: meters/second
     * - Arm: radians/second
     * - Elevator: meters/second
     * 
     * @return Current velocity in configured units per second
     */
    public double getRelativeEncoderVelocity() {
        return this.relativeEncoder.getVelocity();
    }

    // ============ BASIC MOTOR CONTROL METHODS ============
    
    /**
     * Sets motor output as a duty cycle (percentage of voltage).
     * 
     * Duty cycle control is simpler but voltage-dependent:
     * - At 12V battery: 0.5 duty cycle = 6V
     * - At 11V battery: 0.5 duty cycle = 5.5V
     * 
     * For consistent behavior across battery voltage, use setVoltage() instead.
     * 
     * @param dutyCycle Motor output from -1.0 (full reverse) to +1.0 (full forward)
     */
    public void setDutyCycle(double dutyCycle) {
        this.sparkMaxMotorController.set(MathUtil.clamp(dutyCycle, -1.0, 1.0));
    }

    /**
     * Sets motor output as an absolute voltage.
     * 
     * Voltage control compensates for battery voltage sag, providing consistent
     * behavior regardless of battery state. This is preferred for autonomous routines
     * and motion profiling.
     * 
     * The SparkMax will automatically compensate for battery voltage to achieve
     * the requested voltage (up to the current battery voltage).
     * 
     * @param voltage Motor voltage from -12.0 (full reverse) to +12.0 (full forward)
     */
    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
        this.sparkMaxMotorController.setVoltage(clampedVoltage);
    }

    /**
     * Resets both PID controllers to clear accumulated integral and previous error.
     * 
     * Call this when:
     * - Starting a new motion profile
     * - Switching control modes
     * - After a mechanism has been moved manually
     * - To clear integral windup
     */
    public void resetProportionalIntegralDerivativeControllers() {
        this.proportionalIntegralDerivativeControllerVoltage.reset();
        this.proportionalIntegralDerivativeControllerDutyCycle.reset();
    }

    /**
     * Stops the motor and deactivates any active motion profile.
     * 
     * This is a safety method that:
     * 1. Stops motor output immediately
     * 2. Cancels any running motion profile
     * 3. Leaves PID state intact (call resetPID if you want to clear it)
     */
    public void stop() {
        this.sparkMaxMotorController.stopMotor();
        this.profileIsActive = false;
    }

    // ============ SIMPLE PID CONTROL (WITHOUT MOTION PROFILES) ============
    
    /**
     * Simple velocity control using only PID feedback (no feedforward).
     * 
     * This is a basic control mode suitable for:
     * - Flywheels
     * - Intake rollers
     * - Mechanisms with minimal load variation
     * 
     * For better performance, use PIDFController() which includes feedforward.
     * 
     * @param desiredVelocity Target velocity in configured units/second
     */
    public void velocityControlVoltage(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidVoltageOutput = this.proportionalIntegralDerivativeControllerVoltage.calculate(
            measuredVelocity, 
            desiredVelocity
        );
        this.setVoltage(pidVoltageOutput);
    }

    /**
     * Simple position control using only PID feedback (no feedforward).
     * 
     * This is a basic control mode that will work but may:
     * - Oscillate around the target
     * - Overshoot the target
     * - Be sensitive to battery voltage
     * 
     * For better performance with motion profiles, use MagicMotionPositionControl().
     * 
     * @param desiredPosition Target position in configured units
     */
    public void positionControlVoltage(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidVoltageOutput = this.proportionalIntegralDerivativeControllerVoltage.calculate(
            measuredPosition, 
            desiredPosition
        );
        this.setVoltage(pidVoltageOutput);
    }

    /**
     * Simple velocity control using duty cycle output (no feedforward).
     * 
     * Similar to velocityControlVoltage() but outputs duty cycle instead of voltage.
     * Less preferred because it's more sensitive to battery voltage.
     * 
     * @param desiredVelocity Target velocity in configured units/second
     */
    public void velocityControlDutyCycle(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidDutyCycleOutput = this.proportionalIntegralDerivativeControllerDutyCycle.calculate(
            measuredVelocity, 
            desiredVelocity
        );
        this.setDutyCycle(pidDutyCycleOutput);
    }

    /**
     * Simple position control using duty cycle output (no feedforward).
     * 
     * Similar to positionControlVoltage() but outputs duty cycle instead of voltage.
     * Less preferred because it's more sensitive to battery voltage.
     * 
     * @param desiredPosition Target position in configured units
     */
    public void positionControlDutyCycle(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidDutyCycleOutput = this.proportionalIntegralDerivativeControllerDutyCycle.calculate(
            measuredPosition, 
            desiredPosition
        );
        this.setDutyCycle(pidDutyCycleOutput);
    }

    // ============ FEEDFORWARD CALCULATION ============
    
    /**
     * Calculates feedforward voltage based on physics model of the mechanism.
     * 
     * Feedforward is "proactive" control that predicts the voltage needed based on
     * desired motion, rather than "reactive" PID which corrects errors after they occur.
     * 
     * Components:
     * 1. Gravity compensation (Kg * cos(angle)): Counteracts gravity for arms/elevators
     * 2. Acceleration (Ka * acceleration): Provides force to accelerate the mass
     * 3. Velocity (Kv * velocity): Overcomes back-EMF and friction while moving
     * 4. Static friction (Ks * sign(velocity)): Overcomes stiction to start moving
     * 
     * For best results, characterize these constants using WPILib's SysId tool.
     * 
     * @param desiredVelocity Target velocity (units/second)
     * @param desiredAcceleration Target acceleration (units/second²)
     * @return Feedforward voltage to achieve the desired motion
     */
    public double calculateFeedForward(double desiredVelocity, double desiredAcceleration) {
        // Update gravity cosine value based on current position
        // For arms: this calculates cos(current_angle) to scale gravity compensation
        this.userConfiguration.kGcos(this.getRelativeEncoderPosition());
        
        double feedforwardVoltage = 0.0;
        
        // Component 1: Gravity compensation
        // For horizontal mechanisms: Kg * 1 = constant gravity
        // For pivoting arms: Kg * cos(angle) = gravity varies with angle
        // When arm is horizontal (90°): cos(90°) = 0, no compensation needed
        // When arm is vertical (0°): cos(0°) = 1, full compensation needed
        feedforwardVoltage += this.userConfiguration.Kg * 
                              Math.cos(this.userConfiguration.kGcos) * 
                              this.userConfiguration.kGcosRatio;
        
        // Component 2: Acceleration compensation
        // F = ma → Voltage proportional to acceleration
        // Larger Ka means more massive mechanism
        feedforwardVoltage += this.userConfiguration.Ka * desiredAcceleration;
        
        // Component 3: Velocity compensation
        // Overcomes back-EMF (voltage generated by spinning motor)
        // and velocity-dependent friction
        feedforwardVoltage += this.userConfiguration.Kv * desiredVelocity;
        
        // Component 4: Static friction compensation
        // Only apply when moving (not when velocity is zero)
        // Uses sign() to apply in correct direction
        if (Math.abs(desiredVelocity) > 0.001) {
            feedforwardVoltage += this.userConfiguration.Ks * Math.signum(desiredVelocity);
        }

        return feedforwardVoltage;
    }

    /**
     * Combined PID + Feedforward controller for velocity tracking.
     * 
     * This is the recommended way to control velocity because it combines:
     * - Feedforward: Provides most of the voltage based on physics
     * - PID Feedback: Corrects small errors and disturbances
     * 
     * The feedforward does the "heavy lifting" while PID handles the "fine tuning".
     * This results in better tracking with less overshoot than PID alone.
     * 
     * @param desiredVelocity Target velocity (units/second)
     * @param desiredAcceleration Target acceleration (units/second²)
     * @return Total voltage (feedforward + feedback) clamped to battery limits
     */
    public double proportionalIntegralDerivativeFeedforwardController(
            double desiredVelocity, 
            double desiredAcceleration) {
        
        // Calculate feedforward based on desired motion
        double feedforwardVoltage = this.calculateFeedForward(desiredVelocity, desiredAcceleration);
        
        // Calculate feedback to correct any tracking error
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidFeedbackVoltage = this.proportionalIntegralDerivativeControllerVoltage.calculate(
            measuredVelocity, 
            desiredVelocity
        );
        
        // Combine feedforward (proactive) + feedback (reactive)
        double totalVoltage = feedforwardVoltage + pidFeedbackVoltage;
        
        return MathUtil.clamp(totalVoltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
    }

    // ============ MOTION PROFILE GENERATION - TRAPEZOIDAL ============
    
    /**
     * Calculates timing for a trapezoidal velocity profile.
     * 
     * A trapezoidal profile has 3 phases:
     * Phase 1 - Acceleration: Velocity increases linearly from 0 to maximum
     * Phase 2 - Cruise: Velocity stays constant at maximum
     * Phase 3 - Deceleration: Velocity decreases linearly from maximum to 0
     * 
     * The profile is called "trapezoidal" because the velocity-time graph looks
     * like a trapezoid. If the distance is short, it becomes a triangle (no cruise).
     * 
     * Advantages:
     * - Simple to calculate and understand
     * - Efficient (reaches target quickly)
     * - Predictable timing
     * 
     * Disadvantages:
     * - Instantaneous acceleration changes (infinite jerk)
     * - Can cause oscillations in flexible mechanisms
     * - Can cause wheel slip in drivetrains
     * 
     * @param startPosition Starting position (units)
     * @param endPosition Target position (units)
     */
    private void calculateTrapezoidalProfile(double startPosition, double endPosition) {
        double distance = Math.abs(endPosition - startPosition);
        double maximumVelocity = this.userConfiguration.CruiseVelocity;
        double maximumAcceleration = this.userConfiguration.TargetAcceleration;
        
        // Safety check: ensure valid parameters
        if (maximumVelocity <= 0 || maximumAcceleration <= 0) {
            this.trapezoidalAccelerationTime = 0;
            this.trapezoidalCruiseTime = 0;
            this.trapezoidalDecelerationTime = 0;
            this.trapezoidalTotalProfileTime = 0;
            return;
        }
        
        // Calculate time to reach maximum velocity
        // v = a*t → t = v/a
        this.trapezoidalAccelerationTime = maximumVelocity / maximumAcceleration;
        
        // Calculate distance traveled during acceleration
        // d = 0.5*a*t²
        double accelerationDistance = 0.5 * maximumAcceleration * 
                                     this.trapezoidalAccelerationTime * 
                                     this.trapezoidalAccelerationTime;
        
        // Total distance for acceleration + deceleration (symmetric)
        double totalAccelerationDecelerationDistance = 2 * accelerationDistance;
        
        // Check if we have room to reach maximum velocity
        if (totalAccelerationDecelerationDistance > distance) {
            // Distance too short: create triangular profile (no cruise phase)
            // Solve: d = 0.5*a*t² for total distance
            // t = sqrt(d/a)
            this.trapezoidalAccelerationTime = Math.sqrt(distance / maximumAcceleration);
            this.trapezoidalCruiseTime = 0;
            this.trapezoidalDecelerationTime = this.trapezoidalAccelerationTime;
        } else {
            // Distance sufficient: create full trapezoidal profile
            double cruiseDistance = distance - totalAccelerationDecelerationDistance;
            this.trapezoidalCruiseTime = cruiseDistance / maximumVelocity;
            this.trapezoidalDecelerationTime = this.trapezoidalAccelerationTime;
        }
        
        // Calculate total time for the profile
        this.trapezoidalTotalProfileTime = this.trapezoidalAccelerationTime + 
                                           this.trapezoidalCruiseTime + 
                                           this.trapezoidalDecelerationTime;
    }
    
    // ============ MOTION PROFILE GENERATION - S-CURVE ============
    
    /**
     * Calculates timing for an S-Curve (jerk-limited) velocity profile.
     * 
     * An S-Curve profile has 7 phases for smooth acceleration control:
     * Phase 1: Positive jerk (acceleration increases from 0 to max)
     * Phase 2: Constant acceleration at maximum
     * Phase 3: Negative jerk (acceleration decreases from max to 0)
     * Phase 4: Constant velocity cruise
     * Phase 5: Negative jerk (deceleration increases from 0 to max)
     * Phase 6: Constant deceleration at maximum
     * Phase 7: Positive jerk (deceleration decreases from max to 0)
     * 
     * The profile is called "S-Curve" because the velocity-time graph has
     * smooth S-shaped transitions instead of sharp corners.
     * 
     * Advantages:
     * - Smooth motion with controlled jerk
     * - Reduces oscillations in flexible mechanisms
     * - Better for tall robots or delicate mechanisms
     * - Reduces wheel slip
     * 
     * Disadvantages:
     * - More complex to calculate
     * - Takes slightly longer than trapezoidal
     * - Requires tuning jerk limit
     * 
     * Jerk is the rate of change of acceleration (third derivative of position).
     * Lower jerk = smoother but slower
     * Higher jerk = approaches trapezoidal profile
     * 
     * @param startPosition Starting position (units)
     * @param endPosition Target position (units)
     */
    private void calculateSCurveProfile(double startPosition, double endPosition) {
        double distance = Math.abs(endPosition - startPosition);
        double maximumVelocity = this.userConfiguration.CruiseVelocity;
        double maximumAcceleration = this.userConfiguration.TargetAcceleration;
        double maximumJerk = this.userConfiguration.TargetJerk;
        
        // Safety check: ensure valid parameters
        if (maximumVelocity <= 0 || maximumAcceleration <= 0 || maximumJerk <= 0) {
            this.sCurvePhase1Time = 0;
            this.sCurvePhase2Time = 0;
            this.sCurvePhase3Time = 0;
            this.sCurvePhase4Time = 0;
            this.sCurvePhase5Time = 0;
            this.sCurvePhase6Time = 0;
            this.sCurvePhase7Time = 0;
            this.sCurveTotalProfileTime = 0;
            return;
        }
        
        // Phase 1: Calculate time to reach maximum acceleration
        // a = j*t → t = a/j
        this.sCurvePhase1Time = maximumAcceleration / maximumJerk;
        
        // Calculate velocity reached during jerk phases only
        // During phase 1: a(t) = j*t
        // v(t) = integral of a(t) = 0.5*j*t²
        // At end of phase 1: v = 0.5*j*t1²
        double velocityFromJerkPhase = 0.5 * maximumAcceleration * this.sCurvePhase1Time;
        
        // Check if we can reach maximum velocity
        // We have two jerk phases contributing velocity: phase 1 and phase 3
        if (2 * velocityFromJerkPhase >= maximumVelocity) {
            // Cannot reach max acceleration: adjust jerk time
            // Need: 2 * (0.5*j*t²) = vmax
            // Solve for t: t = sqrt(vmax/j)
            this.sCurvePhase1Time = Math.sqrt(maximumVelocity / maximumJerk);
            this.sCurvePhase2Time = 0;  // No constant acceleration phase
            this.sCurvePhase3Time = this.sCurvePhase1Time;  // Symmetric
        } else {
            // Can reach max acceleration: calculate constant acceleration time
            double remainingVelocity = maximumVelocity - 2 * velocityFromJerkPhase;
            this.sCurvePhase2Time = remainingVelocity / maximumAcceleration;
            this.sCurvePhase3Time = this.sCurvePhase1Time;  // Symmetric
        }
        
        // Calculate distances traveled during acceleration phases
        
        // Phase 1 distance: integrate velocity
        // v(t) = 0.5*j*t²
        // d(t) = integral of v(t) = (1/6)*j*t³
        double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                this.sCurvePhase1Time * 
                                this.sCurvePhase1Time * 
                                this.sCurvePhase1Time;
        
        // Velocity at end of phase 1
        double phase1Velocity = 0.5 * maximumJerk * 
                                this.sCurvePhase1Time * 
                                this.sCurvePhase1Time;
        
        // Phase 2 distance: constant acceleration
        // v(t) = v1 + a*t
        // d(t) = v1*t + 0.5*a*t²
        double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                0.5 * maximumAcceleration * 
                                this.sCurvePhase2Time * 
                                this.sCurvePhase2Time;
        
        // Velocity at end of phase 2
        double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
        
        // Phase 3 distance: deceleration of acceleration (jerk negative)
        // Similar to phase 1 but with existing velocity
        double phase3Distance = phase2Velocity * this.sCurvePhase3Time + 
                                0.5 * maximumAcceleration * 
                                this.sCurvePhase3Time * 
                                this.sCurvePhase3Time - 
                                (1.0 / 6.0) * maximumJerk * 
                                this.sCurvePhase3Time * 
                                this.sCurvePhase3Time * 
                                this.sCurvePhase3Time;
        
        // Total distance during acceleration (phases 1-3)
        double totalAccelerationDistance = phase1Distance + phase2Distance + phase3Distance;
        
        // Calculate cruise phase
        // Deceleration is symmetric to acceleration, so total accel+decel distance is:
        double totalAccelerationDecelerationDistance = 2 * totalAccelerationDistance;
        
        if (totalAccelerationDecelerationDistance > distance) {
            // Distance too short for cruise phase
            this.sCurvePhase4Time = 0;
            
            // Scale down the profile to fit the distance
            // This is a simplification - exact solution requires solving cubic equation
            double scaleFactor = Math.sqrt(distance / totalAccelerationDecelerationDistance);
            this.sCurvePhase1Time *= scaleFactor;
            this.sCurvePhase2Time *= scaleFactor;
            this.sCurvePhase3Time *= scaleFactor;
        } else {
            // Sufficient distance for cruise
            double cruiseDistance = distance - totalAccelerationDecelerationDistance;
            
            // Final velocity at end of acceleration
            double finalAccelerationVelocity = phase2Velocity + 
                                               maximumAcceleration * this.sCurvePhase3Time - 
                                               0.5 * maximumJerk * 
                                               this.sCurvePhase3Time * 
                                               this.sCurvePhase3Time;
            
            this.sCurvePhase4Time = cruiseDistance / finalAccelerationVelocity;
        }
        
        // Deceleration phases are symmetric to acceleration
        this.sCurvePhase5Time = this.sCurvePhase3Time;
        this.sCurvePhase6Time = this.sCurvePhase2Time;
        this.sCurvePhase7Time = this.sCurvePhase1Time;
        
        // Calculate total profile time
        this.sCurveTotalProfileTime = this.sCurvePhase1Time + 
                                      this.sCurvePhase2Time + 
                                      this.sCurvePhase3Time + 
                                      this.sCurvePhase4Time + 
                                      this.sCurvePhase5Time + 
                                      this.sCurvePhase6Time + 
                                      this.sCurvePhase7Time;
    }
    
    // ============ MOTION PROFILE STATE CALCULATION - TRAPEZOIDAL ============
    
    /**
     * Calculates the desired state (position, velocity, acceleration) at a given time
     * for a trapezoidal motion profile.
     * 
     * This method determines which phase the profile is in and calculates the
     * kinematic state using the appropriate equations for that phase.
     * 
     * @param time Time since profile start (seconds)
     * @param startPosition Starting position of the profile (units)
     * @param endPosition Target position of the profile (units)
     * @return SparkMaxMotionState containing position, velocity, acceleration, and jerk
     */
    private SparkMaxMotionState getTrapezoidalState(double time, double startPosition, double endPosition) {
        // Determine direction of motion
        double direction = Math.signum(endPosition - startPosition);
        
        double maximumVelocity = this.userConfiguration.CruiseVelocity;
        double maximumAcceleration = this.userConfiguration.TargetAcceleration;
        
        double position, velocity, acceleration;
        
        // Before profile starts
        if (time < 0) {
            return new SparkMaxMotionState(startPosition, 0, 0, 0);
        }
        
        // Phase 1: Acceleration
        else if (time < this.trapezoidalAccelerationTime) {
            // Kinematic equations for constant acceleration:
            // a(t) = a_max (constant)
            // v(t) = a*t (starts from 0)
            // x(t) = x0 + 0.5*a*t²
            
            acceleration = maximumAcceleration;
            velocity = maximumAcceleration * time;
            position = startPosition + direction * 0.5 * maximumAcceleration * time * time;
        }
        
        // Phase 2: Cruise (constant velocity)
        else if (time < this.trapezoidalAccelerationTime + this.trapezoidalCruiseTime) {
            double timeInCruisePhase = time - this.trapezoidalAccelerationTime;
            
            // Distance traveled during acceleration
            double accelerationDistance = 0.5 * maximumAcceleration * 
                                         this.trapezoidalAccelerationTime * 
                                         this.trapezoidalAccelerationTime;
            
            // Velocity reached during acceleration (or peak velocity if triangular)
            double cruiseVelocity = (this.trapezoidalCruiseTime > 0) ? 
                                   maximumVelocity : 
                                   maximumAcceleration * this.trapezoidalAccelerationTime;
            
            // During cruise: no acceleration, constant velocity
            acceleration = 0;
            velocity = cruiseVelocity;
            position = startPosition + direction * (accelerationDistance + cruiseVelocity * timeInCruisePhase);
        }
        
        // Phase 3: Deceleration
        else if (time < this.trapezoidalTotalProfileTime) {
            double timeInDecelerationPhase = time - this.trapezoidalAccelerationTime - this.trapezoidalCruiseTime;
            
            // Distances from previous phases
            double accelerationDistance = 0.5 * maximumAcceleration * 
                                         this.trapezoidalAccelerationTime * 
                                         this.trapezoidalAccelerationTime;
            double cruiseDistance = (this.trapezoidalCruiseTime > 0) ? 
                                   maximumVelocity * this.trapezoidalCruiseTime : 
                                   0;
            double cruiseVelocity = (this.trapezoidalCruiseTime > 0) ? 
                                   maximumVelocity : 
                                   maximumAcceleration * this.trapezoidalAccelerationTime;
            
            // Deceleration phase: negative acceleration
            // v(t) = v_cruise - a*t
            // x(t) = x_start + v_cruise*t - 0.5*a*t²
            
            acceleration = -maximumAcceleration;
            velocity = cruiseVelocity - maximumAcceleration * timeInDecelerationPhase;
            position = startPosition + direction * 
                      (accelerationDistance + cruiseDistance + 
                       cruiseVelocity * timeInDecelerationPhase - 
                       0.5 * maximumAcceleration * timeInDecelerationPhase * timeInDecelerationPhase);
        }
        
        // After profile completes
        else {
            return new SparkMaxMotionState(endPosition, 0, 0, 0);
        }
        
        // Apply direction to velocity and acceleration
        return new SparkMaxMotionState(
            position,
            direction * velocity,
            direction * acceleration,
            0  // Trapezoidal has undefined jerk (infinite at transitions)
        );
    }
    
    // ============ MOTION PROFILE STATE CALCULATION - S-CURVE ============
    
    /**
     * Calculates the desired state (position, velocity, acceleration, jerk) at a given time
     * for an S-Curve motion profile.
     * 
     * This method determines which of the 7 phases the profile is in and calculates
     * the kinematic state using the appropriate equations for that phase.
     * 
     * The math is more complex than trapezoidal because we're integrating jerk
     * to get acceleration, acceleration to get velocity, and velocity to get position.
     * 
     * @param time Time since profile start (seconds)
     * @param startPosition Starting position of the profile (units)
     * @param endPosition Target position of the profile (units)
     * @return SparkMaxMotionState containing position, velocity, acceleration, and jerk
     */
    private SparkMaxMotionState getSCurveState(double time, double startPosition, double endPosition) {
        // Determine direction of motion
        double direction = Math.signum(endPosition - startPosition);
        
        double maximumAcceleration = this.userConfiguration.TargetAcceleration;
        double maximumJerk = this.userConfiguration.TargetJerk;
        
        double position, velocity, acceleration, jerk;
        
        // Before profile starts
        if (time < 0) {
            return new SparkMaxMotionState(startPosition, 0, 0, 0);
        }
        
        // ===== ACCELERATION SECTION =====
        
        // Phase 1: Positive jerk (acceleration increasing from 0 to max)
        else if (time < this.sCurvePhase1Time) {
            // Jerk is constant and positive
            // a(t) = j*t (integration of jerk)
            // v(t) = 0.5*j*t² (integration of acceleration)
            // x(t) = (1/6)*j*t³ (integration of velocity)
            
            jerk = maximumJerk;
            acceleration = maximumJerk * time;
            velocity = 0.5 * maximumJerk * time * time;
            position = startPosition + direction * (1.0 / 6.0) * maximumJerk * time * time * time;
        }
        
        // Phase 2: Constant maximum acceleration
        else if (time < this.sCurvePhase1Time + this.sCurvePhase2Time) {
            double timeInPhase2 = time - this.sCurvePhase1Time;
            
            // State at end of phase 1
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            // Constant acceleration phase
            // a(t) = a_max (constant)
            // v(t) = v1 + a*t
            // x(t) = x1 + v1*t + 0.5*a*t²
            
            jerk = 0;
            acceleration = maximumAcceleration;
            velocity = phase1Velocity + maximumAcceleration * timeInPhase2;
            position = startPosition + direction * 
                      (phase1Distance + phase1Velocity * timeInPhase2 + 
                       0.5 * maximumAcceleration * timeInPhase2 * timeInPhase2);
        }
        
        // Phase 3: Negative jerk (acceleration decreasing from max to 0)
        else if (time < this.sCurvePhase1Time + this.sCurvePhase2Time + this.sCurvePhase3Time) {
            double timeInPhase3 = time - this.sCurvePhase1Time - this.sCurvePhase2Time;
            
            // State at end of phase 1
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            // State at end of phase 2
            double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase2Time * 
                                   this.sCurvePhase2Time;
            double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
            
            // Negative jerk phase (deceleration of acceleration)
            // j(t) = -j_max
            // a(t) = a_max - j*t
            // v(t) = v2 + a_max*t - 0.5*j*t²
            // x(t) = x2 + v2*t + 0.5*a_max*t² - (1/6)*j*t³
            
            jerk = -maximumJerk;
            acceleration = maximumAcceleration - maximumJerk * timeInPhase3;
            velocity = phase2Velocity + maximumAcceleration * timeInPhase3 - 
                      0.5 * maximumJerk * timeInPhase3 * timeInPhase3;
            position = startPosition + direction * 
                      (phase1Distance + phase2Distance + 
                       phase2Velocity * timeInPhase3 + 
                       0.5 * maximumAcceleration * timeInPhase3 * timeInPhase3 - 
                       (1.0 / 6.0) * maximumJerk * timeInPhase3 * timeInPhase3 * timeInPhase3);
        }
        
        // ===== CRUISE SECTION =====
        
        // Phase 4: Constant velocity cruise
        else if (time < this.sCurvePhase1Time + this.sCurvePhase2Time + 
                        this.sCurvePhase3Time + this.sCurvePhase4Time) {
            double timeInPhase4 = time - this.sCurvePhase1Time - 
                                 this.sCurvePhase2Time - 
                                 this.sCurvePhase3Time;
            
            // Calculate state at end of acceleration (end of phase 3)
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase2Time * 
                                   this.sCurvePhase2Time;
            double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
            
            double phase3Distance = phase2Velocity * this.sCurvePhase3Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            double finalVelocity = phase2Velocity + maximumAcceleration * this.sCurvePhase3Time - 
                                  0.5 * maximumJerk * 
                                  this.sCurvePhase3Time * 
                                  this.sCurvePhase3Time;
            
            // Cruise phase: constant velocity, no acceleration or jerk
            jerk = 0;
            acceleration = 0;
            velocity = finalVelocity;
            position = startPosition + direction * 
                      (phase1Distance + phase2Distance + phase3Distance + 
                       finalVelocity * timeInPhase4);
        }
        
        // ===== DECELERATION SECTION (symmetric to acceleration) =====
        
        // Phase 5: Negative jerk (deceleration increasing from 0 to max)
        else if (time < this.sCurvePhase1Time + this.sCurvePhase2Time + 
                        this.sCurvePhase3Time + this.sCurvePhase4Time + 
                        this.sCurvePhase5Time) {
            double timeInPhase5 = time - this.sCurvePhase1Time - 
                                 this.sCurvePhase2Time - 
                                 this.sCurvePhase3Time - 
                                 this.sCurvePhase4Time;
            
            // Calculate cumulative distance and velocity up to cruise
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase2Time * 
                                   this.sCurvePhase2Time;
            double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
            
            double phase3Distance = phase2Velocity * this.sCurvePhase3Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            double cruiseVelocity = phase2Velocity + maximumAcceleration * this.sCurvePhase3Time - 
                                   0.5 * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            
            double phase4Distance = cruiseVelocity * this.sCurvePhase4Time;
            
            // Phase 5: Start of deceleration with negative jerk
            // j(t) = -j_max
            // a(t) = -j*t (deceleration building up)
            // v(t) = v_cruise - 0.5*j*t²
            // x(t) = x4 + v_cruise*t - (1/6)*j*t³
            
            jerk = -maximumJerk;
            acceleration = -maximumJerk * timeInPhase5;
            velocity = cruiseVelocity - 0.5 * maximumJerk * timeInPhase5 * timeInPhase5;
            position = startPosition + direction * 
                      (phase1Distance + phase2Distance + phase3Distance + phase4Distance + 
                       cruiseVelocity * timeInPhase5 - 
                       (1.0 / 6.0) * maximumJerk * timeInPhase5 * timeInPhase5 * timeInPhase5);
        }
        
        // Phase 6: Constant maximum deceleration
        else if (time < this.sCurvePhase1Time + this.sCurvePhase2Time + 
                        this.sCurvePhase3Time + this.sCurvePhase4Time + 
                        this.sCurvePhase5Time + this.sCurvePhase6Time) {
            double timeInPhase6 = time - this.sCurvePhase1Time - 
                                 this.sCurvePhase2Time - 
                                 this.sCurvePhase3Time - 
                                 this.sCurvePhase4Time - 
                                 this.sCurvePhase5Time;
            
            // Calculate all previous distances and velocities
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase2Time * 
                                   this.sCurvePhase2Time;
            double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
            
            double phase3Distance = phase2Velocity * this.sCurvePhase3Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            double cruiseVelocity = phase2Velocity + maximumAcceleration * this.sCurvePhase3Time - 
                                   0.5 * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            
            double phase4Distance = cruiseVelocity * this.sCurvePhase4Time;
            
            double phase5Distance = cruiseVelocity * this.sCurvePhase5Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time;
            double phase5Velocity = cruiseVelocity - 0.5 * maximumJerk * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time;
            
            // Phase 6: Constant deceleration
            // j(t) = 0
            // a(t) = -a_max
            // v(t) = v5 - a_max*t
            // x(t) = x5 + v5*t - 0.5*a_max*t²
            
            jerk = 0;
            acceleration = -maximumAcceleration;
            velocity = phase5Velocity - maximumAcceleration * timeInPhase6;
            position = startPosition + direction * 
                      (phase1Distance + phase2Distance + phase3Distance + 
                       phase4Distance + phase5Distance + 
                       phase5Velocity * timeInPhase6 - 
                       0.5 * maximumAcceleration * timeInPhase6 * timeInPhase6);
        }
        
        // Phase 7: Positive jerk (deceleration decreasing from max to 0)
        else if (time < this.sCurveTotalProfileTime) {
            double timeInPhase7 = time - this.sCurvePhase1Time - 
                                 this.sCurvePhase2Time - 
                                 this.sCurvePhase3Time - 
                                 this.sCurvePhase4Time - 
                                 this.sCurvePhase5Time - 
                                 this.sCurvePhase6Time;
            
            // Calculate all previous distances and velocities
            double phase1Distance = (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            double phase1Velocity = 0.5 * maximumJerk * 
                                   this.sCurvePhase1Time * 
                                   this.sCurvePhase1Time;
            
            double phase2Distance = phase1Velocity * this.sCurvePhase2Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase2Time * 
                                   this.sCurvePhase2Time;
            double phase2Velocity = phase1Velocity + maximumAcceleration * this.sCurvePhase2Time;
            
            double phase3Distance = phase2Velocity * this.sCurvePhase3Time + 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            double cruiseVelocity = phase2Velocity + maximumAcceleration * this.sCurvePhase3Time - 
                                   0.5 * maximumJerk * 
                                   this.sCurvePhase3Time * 
                                   this.sCurvePhase3Time;
            
            double phase4Distance = cruiseVelocity * this.sCurvePhase4Time;
            
            double phase5Distance = cruiseVelocity * this.sCurvePhase5Time - 
                                   (1.0 / 6.0) * maximumJerk * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time;
            double phase5Velocity = cruiseVelocity - 0.5 * maximumJerk * 
                                   this.sCurvePhase5Time * 
                                   this.sCurvePhase5Time;
            
            double phase6Distance = phase5Velocity * this.sCurvePhase6Time - 
                                   0.5 * maximumAcceleration * 
                                   this.sCurvePhase6Time * 
                                   this.sCurvePhase6Time;
            double phase6Velocity = phase5Velocity - maximumAcceleration * this.sCurvePhase6Time;
            
            // Phase 7: Positive jerk bringing deceleration to zero
            // j(t) = +j_max
            // a(t) = -a_max + j*t
            // v(t) = v6 - a_max*t + 0.5*j*t²
            // x(t) = x6 + v6*t - 0.5*a_max*t² + (1/6)*j*t³
            
            jerk = maximumJerk;
            acceleration = -maximumAcceleration + maximumJerk * timeInPhase7;
            velocity = phase6Velocity - maximumAcceleration * timeInPhase7 + 
                      0.5 * maximumJerk * timeInPhase7 * timeInPhase7;
            position = startPosition + direction * 
                      (phase1Distance + phase2Distance + phase3Distance + 
                       phase4Distance + phase5Distance + phase6Distance + 
                       phase6Velocity * timeInPhase7 - 
                       0.5 * maximumAcceleration * timeInPhase7 * timeInPhase7 + 
                       (1.0 / 6.0) * maximumJerk * timeInPhase7 * timeInPhase7 * timeInPhase7);
        }
        
        // After profile completes
        else {
            return new SparkMaxMotionState(endPosition, 0, 0, 0);
        }
        
        // Apply direction to velocity, acceleration, and jerk
        return new SparkMaxMotionState(
            position,
            direction * velocity,
            direction * acceleration,
            direction * jerk
        );
    }

    // ============ HIGH-LEVEL MOTION PROFILE CONTROL ============

    /**
     * Calculates and applies control output for motion profile tracking.
     * 
     * This is the main "brain" of the motion profile system. It:
     * 1. Checks if a new profile needs to be started
     * 2. Calculates the desired state from the profile
     * 3. Combines feedforward (based on desired motion) + feedback (error correction)
     * 4. Applies the combined voltage to the motor
     * 
     * Call this method repeatedly (every 20ms) from periodic() to track a profile.
     * 
     * @param setPoint Target position or velocity (depending on isPositionControl)
     * @param currentSetpoint Starting position or velocity for the profile
     * @param useSCurveProfile true = S-Curve (7 phases), false = Trapezoidal (3 phases)
     * @param isPositionControl true = position control, false = velocity control
     * @return Calculated voltage to apply to motor (already clamped to ±12V)
     */
    public double calculateProfileMotion(double setPoint, 
                                        double currentSetpoint, 
                                        boolean useSCurveProfile, 
                                        boolean isPositionControl) {
        
        // Check if we need to start a new profile
        // Start new if: no active profile OR target changed significantly
        if (!this.profileIsActive || Math.abs(this.profileTargetPosition - setPoint) > 0.001) {
            initializeProfile(currentSetpoint, setPoint, useSCurveProfile);
        }
        
        // Calculate time elapsed since profile started
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - this.profileStartTime;
        
        // Get desired state from the motion profile
        SparkMaxMotionState desiredState;
        if (this.profileUsesJerkControl) {
            desiredState = getSCurveState(elapsedTime, this.profileStartPosition, this.profileTargetPosition);
        } else {
            desiredState = getTrapezoidalState(elapsedTime, this.profileStartPosition, this.profileTargetPosition);
        }
        
        // Update current profile state for telemetry
        this.currentProfilePosition = desiredState.position;
        this.currentProfileVelocity = desiredState.velocity;
        this.currentProfileAcceleration = desiredState.acceleration;
        this.currentProfileJerk = desiredState.jerk;
        
        // Calculate feedforward: voltage based on desired motion
        double feedforwardVoltage = this.calculateFeedForward(
            desiredState.velocity,
            desiredState.acceleration
        );
        
        // Calculate feedback: voltage to correct tracking error
        double feedbackVoltage;
        if (isPositionControl) {
            // Position control: compare actual position to desired position
            double currentPosition = this.getRelativeEncoderPosition();
            feedbackVoltage = this.proportionalIntegralDerivativeControllerVoltage.calculate(
                currentPosition, 
                desiredState.position
            );
        } else {
            // Velocity control: compare actual velocity to desired velocity
            double currentVelocity = this.getRelativeEncoderVelocity();
            feedbackVoltage = this.proportionalIntegralDerivativeControllerVoltage.calculate(
                currentVelocity, 
                desiredState.velocity
            );
        }
        
        // Combine feedforward (proactive) + feedback (reactive) control
        double totalVoltage = feedforwardVoltage + feedbackVoltage;
        
        // Check if profile has finished
        double profileDuration = this.profileUsesJerkControl ? 
                                this.sCurveTotalProfileTime : 
                                this.trapezoidalTotalProfileTime;
        
        if (elapsedTime >= profileDuration) {
            this.profileIsActive = false;
        }
        
        return MathUtil.clamp(totalVoltage, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
    }
    
    /**
     * Initializes a new motion profile.
     * 
     * This method:
     * 1. Records start position, target position, and start time
     * 2. Resets PID to clear old state
     * 3. Calculates profile timing (trapezoidal or S-curve)
     * 4. Marks profile as active
     * 
     * @param startPosition Starting position for the profile
     * @param endPosition Target position for the profile
     * @param useSCurve true = S-Curve profile, false = Trapezoidal profile
     */
    private void initializeProfile(double startPosition, double endPosition, boolean useSCurve) {
        this.profileStartPosition = startPosition;
        this.profileTargetPosition = endPosition;
        this.profileStartTime = System.currentTimeMillis() / 1000.0;
        this.profileIsActive = true;
        this.profileUsesJerkControl = useSCurve;
        
        // Reset PID to prevent integral windup from previous motion
        this.resetProportionalIntegralDerivativeControllers();
        
        // Calculate the profile timing
        if (useSCurve) {
            calculateSCurveProfile(startPosition, endPosition);
        } else {
            calculateTrapezoidalProfile(startPosition, endPosition);
        }
    }

    // ============ USER-FRIENDLY CONTROL METHODS ============

    /**
     * Moves to a target position using a trapezoidal motion profile.
     * 
     * This is the simplest way to use motion profiles. Just call this method
     * repeatedly from periodic() and it will smoothly move to the target.
     * 
     * Example:
     * ```
     * @Override
     * public void periodic() {
     *     motor.magicMotionPositionControl(targetPosition);
     * }
     * ```
     * 
     * @param desiredPosition Target position in configured units
     */
    public void magicMotionPositionControl(double desiredPosition) {
        magicMotionPositionControl(desiredPosition, false);
    }
    
    /**
     * Moves to a target position using a motion profile.
     * 
     * Choose between trapezoidal (faster) or S-curve (smoother) profiles.
     * 
     * Use trapezoidal for:
     * - Drivetrain
     * - Fast movements
     * - Robust mechanisms
     * 
     * Use S-curve for:
     * - Tall arms
     * - Elevators with extension
     * - Mechanisms prone to oscillation
     * 
     * @param desiredPosition Target position in configured units
     * @param enableJerkControl true = S-Curve, false = Trapezoidal
     */
    public void magicMotionPositionControl(double desiredPosition, boolean enableJerkControl) {
        double currentPosition = this.getRelativeEncoderPosition();
        double voltage = calculateProfileMotion(desiredPosition, currentPosition, enableJerkControl, true);
        this.setVoltage(voltage);
    }

    /**
     * Accelerates to a target velocity using a trapezoidal profile.
     * 
     * This smoothly ramps velocity instead of commanding it instantly.
     * Useful for flywheels and other velocity-controlled mechanisms.
     * 
     * @param desiredVelocity Target velocity in configured units/second
     */
    public void magicMotionVelocityControl(double desiredVelocity) {
        magicMotionVelocityControl(desiredVelocity, false);
    }

    /**
     * Accelerates to a target velocity using a motion profile.
     * 
     * @param desiredVelocity Target velocity in configured units/second
     * @param enableJerkControl true = S-Curve, false = Trapezoidal
     */
    public void magicMotionVelocityControl(double desiredVelocity, boolean enableJerkControl) {
        double currentVelocity = this.getRelativeEncoderVelocity();
        double voltage = calculateProfileMotion(desiredVelocity, currentVelocity, enableJerkControl, false);
        this.setVoltage(voltage);
    }

    // ============ MOTION PROFILE STATUS AND TELEMETRY ============
    
    /**
     * Checks if the current motion profile has finished.
     * 
     * Use this in command isFinished() methods:
     * ```
     * @Override
     * public boolean isFinished() {
     *     return motor.isProfileFinished();
     * }
     * ```
     * 
     * @return true if profile is complete or no profile is active
     */
    public boolean isProfileFinished() {
        return !this.profileIsActive;
    }
    
    /**
     * Gets the current desired position from the motion profile.
     * 
     * This is NOT the encoder position - it's where the profile wants to be.
     * Compare this to actual encoder position to see tracking error.
     * 
     * @return Profile's current target position
     */
    public double getCurrentProfilePosition() {
        return this.currentProfilePosition;
    }
    
    /**
     * Gets the current desired velocity from the motion profile.
     * 
     * @return Profile's current target velocity
     */
    public double getCurrentProfileVelocity() {
        return this.currentProfileVelocity;
    }
    
    /**
     * Gets the current desired acceleration from the motion profile.
     * 
     * @return Profile's current target acceleration
     */
    public double getCurrentProfileAcceleration() {
        return this.currentProfileAcceleration;
    }
    
    /**
     * Gets the current jerk from the motion profile.
     * 
     * Only meaningful for S-Curve profiles.
     * Will be 0 for trapezoidal profiles.
     * 
     * @return Profile's current jerk (0 for trapezoidal)
     */
    public double getCurrentProfileJerk() {
        return this.currentProfileJerk;
    }
    
    /**
     * Immediately cancels the current motion profile and stops the motor.
     * 
     * Use this for emergency stops or when switching to manual control.
     */
    public void cancelProfile() {
        this.profileIsActive = false;
        this.stop();
    }
    
    /**
     * Gets the total planned duration of the current profile.
     * 
     * @return Total profile time in seconds, or 0 if no active profile
     */
    public double getProfileTotalTime() {
        return this.profileUsesJerkControl ? 
               this.sCurveTotalProfileTime : 
               this.trapezoidalTotalProfileTime;
    }
    
    /**
     * Gets the time elapsed since the profile started.
     * 
     * @return Elapsed time in seconds, or 0 if no active profile
     */
    public double getProfileElapsedTime() {
        if (!this.profileIsActive) {
            return 0;
        }
        double currentTime = System.currentTimeMillis() / 1000.0;
        return currentTime - this.profileStartTime;
    }
    
    /**
     * Checks if a motion profile is currently running.
     * 
     * @return true if profile is active, false otherwise
     */
    public boolean isProfileActive() {
        return this.profileIsActive;
    }
}