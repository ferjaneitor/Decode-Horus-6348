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

    // ============ MOTION PROFILE STATE ============
    // Tracking del perfil actual
    private double profileStartPosition;
    private double profileTargetPosition;
    private double profileStartTime;
    private boolean profileActive;
    private boolean useJerkControl;
    
    // Estado actual del perfil calculado
    private double currentProfilePosition;
    private double currentProfileVelocity;
    private double currentProfileAcceleration;
    private double currentProfileJerk;
    
    // Tiempos calculados del perfil
    private double accelTime;
    private double cruiseTime;
    private double decelTime;
    private double totalProfileTime;
    
    // Para S-Curve: tiempos de las 7 fases
    private double t1, t2, t3, t4, t5, t6, t7;

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
        this.relativeEncoder = this.sparkMaxMotorController.getEncoder();

        this.userConfiguration = userConfiguration;
        this.revConfiguration = buildRevConfigurationFromUser(userConfiguration);

        this.sparkMaxMotorController.configure(
                this.revConfiguration,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
        );

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
        
        // Inicializar estado del perfil
        this.profileActive = false;
        this.useJerkControl = false;
        this.profileStartPosition = 0;
        this.profileTargetPosition = 0;
        this.profileStartTime = 0;
        this.currentProfilePosition = 0;
        this.currentProfileVelocity = 0;
        this.currentProfileAcceleration = 0;
        this.currentProfileJerk = 0;
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

    // ============ ENCODER METHODS ============
    
    public double getRelativeEncoderPosition() {
        return this.relativeEncoder.getPosition();
    }

    public double getRelativeEncoderVelocity() {
        return this.relativeEncoder.getVelocity();
    }

    // ============ BASIC CONTROL METHODS ============
    
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

    public void stop() {
        this.sparkMaxMotorController.stopMotor();
        this.profileActive = false;
    }

    // ============ SIMPLE PID CONTROL (WITHOUT PROFILES) ============
    
    public void velocityControlVoltage(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredVelocity, desiredVelocity);
        this.setVoltage(pidVoltageOutput);
    }

    public void positionControlVoltage(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredPosition, desiredPosition);
        this.setVoltage(pidVoltageOutput);
    }

    public void velocityControlDutyCycle(double desiredVelocity) {
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidDutyCycleOutput = this.pidControllerDutyCycle.calculate(measuredVelocity, desiredVelocity);
        this.setDutyCycle(pidDutyCycleOutput);
    }

    public void positionControlDutyCycle(double desiredPosition) {
        double measuredPosition = this.getRelativeEncoderPosition();
        double pidDutyCycleOutput = this.pidControllerDutyCycle.calculate(measuredPosition, desiredPosition);
        this.setDutyCycle(pidDutyCycleOutput);
    }

    // ============ FEEDFORWARD CALCULATION ============
    
    /**
     * Calcula el feedforward basado en la velocidad y aceleración deseadas
     * Incluye compensación gravitacional para brazos/elevadores
     */
    public double calculateFeedForward(double desiredVelocity, double desiredAcceleration) {
        this.userConfiguration.kGcos(this.getRelativeEncoderPosition());
        
        double feedforward = 0.0;
        
        // Compensación gravitacional (brazos pivotantes o elevadores)
        feedforward += this.userConfiguration.Kg * Math.cos(this.userConfiguration.kGcos) * this.userConfiguration.kGcosRatio;
        
        // Componente de aceleración
        feedforward += this.userConfiguration.Ka * desiredAcceleration;
        
        // Componente de velocidad
        feedforward += this.userConfiguration.Kv * desiredVelocity;
        
        // Fricción estática
        if (Math.abs(desiredVelocity) > 0.001) {
            feedforward += this.userConfiguration.Ks * Math.signum(desiredVelocity);
        }

        return feedforward;
    }

    /**
     * Control PID + Feedforward para seguimiento de velocidad
     */
    public double PIDFController(double desiredVelocity, double desiredAcceleration) {
        double feedforward = this.calculateFeedForward(desiredVelocity, desiredAcceleration);
        double measuredVelocity = this.getRelativeEncoderVelocity();
        double pidVoltageOutput = this.pidControllerVoltage.calculate(measuredVelocity, desiredVelocity);
        double output = feedforward + pidVoltageOutput;
        return MathUtil.clamp(output, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
    }

    // ============ MOTION PROFILE GENERATION ============
    
    /**
     * Calcula los tiempos del perfil trapezoidal
     */
    private void calculateTrapezoidalProfile(double startPos, double endPos) {
        double distance = Math.abs(endPos - startPos);
        double maxVel = this.userConfiguration.CruiseVelocity;
        double maxAccel = this.userConfiguration.TargetAcceleration;
        
        if (maxVel <= 0 || maxAccel <= 0) {
            this.accelTime = 0;
            this.cruiseTime = 0;
            this.decelTime = 0;
            this.totalProfileTime = 0;
            return;
        }
        
        // Tiempo para alcanzar velocidad máxima
        this.accelTime = maxVel / maxAccel;
        
        // Distancia durante aceleración y desaceleración
        double accelDistance = 0.5 * maxAccel * this.accelTime * this.accelTime;
        double totalAccelDecel = 2 * accelDistance;
        
        if (totalAccelDecel > distance) {
            // Perfil triangular (no hay fase de crucero)
            this.accelTime = Math.sqrt(distance / maxAccel);
            this.cruiseTime = 0;
            this.decelTime = this.accelTime;
        } else {
            // Perfil trapezoidal completo
            double cruiseDistance = distance - totalAccelDecel;
            this.cruiseTime = cruiseDistance / maxVel;
            this.decelTime = this.accelTime;
        }
        
        this.totalProfileTime = this.accelTime + this.cruiseTime + this.decelTime;
    }
    
    /**
     * Calcula los tiempos del perfil S-Curve (7 fases)
     */
    private void calculateSCurveProfile(double startPos, double endPos) {
        double distance = Math.abs(endPos - startPos);
        double maxVel = this.userConfiguration.CruiseVelocity;
        double maxAccel = this.userConfiguration.TargetAcceleration;
        double maxJerk = this.userConfiguration.TargetJerk;
        
        if (maxVel <= 0 || maxAccel <= 0 || maxJerk <= 0) {
            this.t1 = this.t2 = this.t3 = this.t4 = this.t5 = this.t6 = this.t7 = 0;
            this.totalProfileTime = 0;
            return;
        }
        
        // Tiempo para alcanzar aceleración máxima (Fase 1)
        this.t1 = maxAccel / maxJerk;
        
        // Velocidad alcanzada solo con jerk
        double vJerk = 0.5 * maxAccel * this.t1;
        
        // Verificar si alcanzamos velocidad máxima
        if (2 * vJerk >= maxVel) {
            // No hay fase de aceleración constante
            this.t1 = Math.sqrt(maxVel / maxJerk);
            this.t2 = 0;
            this.t3 = this.t1;
        } else {
            // Sí hay fase de aceleración constante
            double remainingVel = maxVel - 2 * vJerk;
            this.t2 = remainingVel / maxAccel;
            this.t3 = this.t1;
        }
        
        // Calcular distancia de aceleración
        double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
        double v1 = 0.5 * maxJerk * this.t1 * this.t1;
        double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
        double v2 = v1 + maxAccel * this.t2;
        double d3 = v2 * this.t3 + 0.5 * maxAccel * this.t3 * this.t3 - (1.0 / 6.0) * maxJerk * this.t3 * this.t3 * this.t3;
        
        double accelDistance = d1 + d2 + d3;
        
        // Fase de crucero
        if (accelDistance * 2 > distance) {
            // No hay fase de crucero, recalcular para distancia corta
            this.t4 = 0;
            // Simplificación: escalar tiempos proporcionalmente
            double scaleFactor = Math.sqrt(distance / (2 * accelDistance));
            this.t1 *= scaleFactor;
            this.t2 *= scaleFactor;
            this.t3 *= scaleFactor;
        } else {
            double cruiseDistance = distance - 2 * accelDistance;
            this.t4 = cruiseDistance / maxVel;
        }
        
        // Fases de desaceleración son simétricas
        this.t5 = this.t3;
        this.t6 = this.t2;
        this.t7 = this.t1;
        
        this.totalProfileTime = this.t1 + this.t2 + this.t3 + this.t4 + this.t5 + this.t6 + this.t7;
    }
    
    /**
     * Obtiene el estado del perfil trapezoidal en un tiempo dado
     */
    private SparkMaxMotionState getTrapezoidalState(double t, double startPos, double endPos) {
        double direction = Math.signum(endPos - startPos);
        double maxVel = this.userConfiguration.CruiseVelocity;
        double maxAccel = this.userConfiguration.TargetAcceleration;
        
        double position, velocity, acceleration;
        
        if (t < 0) {
            return new SparkMaxMotionState(startPos, 0, 0, 0);
        } else if (t < this.accelTime) {
            // Fase 1: Aceleración
            acceleration = maxAccel;
            velocity = maxAccel * t;
            position = startPos + direction * 0.5 * maxAccel * t * t;
        } else if (t < this.accelTime + this.cruiseTime) {
            // Fase 2: Crucero
            double dt = t - this.accelTime;
            double accelDist = 0.5 * maxAccel * this.accelTime * this.accelTime;
            double cruiseVel = (this.cruiseTime > 0) ? maxVel : maxAccel * this.accelTime;
            
            acceleration = 0;
            velocity = cruiseVel;
            position = startPos + direction * (accelDist + cruiseVel * dt);
        } else if (t < this.totalProfileTime) {
            // Fase 3: Desaceleración
            double dt = t - this.accelTime - this.cruiseTime;
            double accelDist = 0.5 * maxAccel * this.accelTime * this.accelTime;
            double cruiseDist = (this.cruiseTime > 0) ? maxVel * this.cruiseTime : 0;
            double cruiseVel = (this.cruiseTime > 0) ? maxVel : maxAccel * this.accelTime;
            
            acceleration = -maxAccel;
            velocity = cruiseVel - maxAccel * dt;
            position = startPos + direction * 
                      (accelDist + cruiseDist + cruiseVel * dt - 0.5 * maxAccel * dt * dt);
        } else {
            return new SparkMaxMotionState(endPos, 0, 0, 0);
        }
        
        return new SparkMaxMotionState(
            position,
            direction * velocity,
            direction * acceleration,
            0
        );
    }
    
    /**
     * Obtiene el estado del perfil S-Curve en un tiempo dado
     */
    private SparkMaxMotionState getSCurveState(double t, double startPos, double endPos) {
        double direction = Math.signum(endPos - startPos);
        double maxVel = this.userConfiguration.CruiseVelocity;
        double maxAccel = this.userConfiguration.TargetAcceleration;
        double maxJerk = this.userConfiguration.TargetJerk;
        
        double position, velocity, acceleration, jerk;
        
        double cumulativeTime = 0;
        
        if (t < 0) {
            return new SparkMaxMotionState(startPos, 0, 0, 0);
        }
        
        // Fase 1: Jerk positivo (aceleración aumenta)
        else if (t < this.t1) {
            jerk = maxJerk;
            acceleration = maxJerk * t;
            velocity = 0.5 * maxJerk * t * t;
            position = startPos + direction * (1.0 / 6.0) * maxJerk * t * t * t;
        }
        
        // Fase 2: Aceleración constante
        else if (t < this.t1 + this.t2) {
            double dt = t - this.t1;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            
            jerk = 0;
            acceleration = maxAccel;
            velocity = v1 + maxAccel * dt;
            position = startPos + direction * (d1 + v1 * dt + 0.5 * maxAccel * dt * dt);
        }
        
        // Fase 3: Jerk negativo (aceleración disminuye a 0)
        else if (t < this.t1 + this.t2 + this.t3) {
            double dt = t - this.t1 - this.t2;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
            double v2 = v1 + maxAccel * this.t2;
            
            jerk = -maxJerk;
            acceleration = maxAccel - maxJerk * dt;
            velocity = v2 + maxAccel * dt - 0.5 * maxJerk * dt * dt;
            position = startPos + direction * 
                      (d1 + d2 + v2 * dt + 0.5 * maxAccel * dt * dt - (1.0 / 6.0) * maxJerk * dt * dt * dt);
        }
        
        // Fase 4: Velocidad constante
        else if (t < this.t1 + this.t2 + this.t3 + this.t4) {
            double dt = t - this.t1 - this.t2 - this.t3;
            
            // Calcular distancia y velocidad al final de la aceleración
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
            double v2 = v1 + maxAccel * this.t2;
            double d3 = v2 * this.t3 + 0.5 * maxAccel * this.t3 * this.t3 - (1.0 / 6.0) * maxJerk * this.t3 * this.t3 * this.t3;
            double vFinal = v2 + maxAccel * this.t3 - 0.5 * maxJerk * this.t3 * this.t3;
            
            jerk = 0;
            acceleration = 0;
            velocity = vFinal;
            position = startPos + direction * (d1 + d2 + d3 + vFinal * dt);
        }
        
        // Fase 5: Jerk negativo (desaceleración aumenta)
        else if (t < this.t1 + this.t2 + this.t3 + this.t4 + this.t5) {
            double dt = t - this.t1 - this.t2 - this.t3 - this.t4;
            
            // Calcular estado al inicio de la desaceleración
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
            double v2 = v1 + maxAccel * this.t2;
            double d3 = v2 * this.t3 + 0.5 * maxAccel * this.t3 * this.t3 - (1.0 / 6.0) * maxJerk * this.t3 * this.t3 * this.t3;
            double vCruise = v2 + maxAccel * this.t3 - 0.5 * maxJerk * this.t3 * this.t3;
            double d4 = vCruise * this.t4;
            
            jerk = -maxJerk;
            acceleration = -maxJerk * dt;
            velocity = vCruise - 0.5 * maxJerk * dt * dt;
            position = startPos + direction * 
                      (d1 + d2 + d3 + d4 + vCruise * dt - (1.0 / 6.0) * maxJerk * dt * dt * dt);
        }
        
        // Fase 6: Desaceleración constante
        else if (t < this.t1 + this.t2 + this.t3 + this.t4 + this.t5 + this.t6) {
            double dt = t - this.t1 - this.t2 - this.t3 - this.t4 - this.t5;
            
            // Estado al final de fase 5
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
            double v2 = v1 + maxAccel * this.t2;
            double d3 = v2 * this.t3 + 0.5 * maxAccel * this.t3 * this.t3 - (1.0 / 6.0) * maxJerk * this.t3 * this.t3 * this.t3;
            double vCruise = v2 + maxAccel * this.t3 - 0.5 * maxJerk * this.t3 * this.t3;
            double d4 = vCruise * this.t4;
            double d5 = vCruise * this.t5 - (1.0 / 6.0) * maxJerk * this.t5 * this.t5 * this.t5;
            double v5 = vCruise - 0.5 * maxJerk * this.t5 * this.t5;
            
            jerk = 0;
            acceleration = -maxAccel;
            velocity = v5 - maxAccel * dt;
            position = startPos + direction * 
                      (d1 + d2 + d3 + d4 + d5 + v5 * dt - 0.5 * maxAccel * dt * dt);
        }
        
        // Fase 7: Jerk positivo (desaceleración disminuye a 0)
        else if (t < this.totalProfileTime) {
            double dt = t - this.t1 - this.t2 - this.t3 - this.t4 - this.t5 - this.t6;
            
            // Estado al final de fase 6
            double d1 = (1.0 / 6.0) * maxJerk * this.t1 * this.t1 * this.t1;
            double v1 = 0.5 * maxJerk * this.t1 * this.t1;
            double d2 = v1 * this.t2 + 0.5 * maxAccel * this.t2 * this.t2;
            double v2 = v1 + maxAccel * this.t2;
            double d3 = v2 * this.t3 + 0.5 * maxAccel * this.t3 * this.t3 - (1.0 / 6.0) * maxJerk * this.t3 * this.t3 * this.t3;
            double vCruise = v2 + maxAccel * this.t3 - 0.5 * maxJerk * this.t3 * this.t3;
            double d4 = vCruise * this.t4;
            double d5 = vCruise * this.t5 - (1.0 / 6.0) * maxJerk * this.t5 * this.t5 * this.t5;
            double v5 = vCruise - 0.5 * maxJerk * this.t5 * this.t5;
            double d6 = v5 * this.t6 - 0.5 * maxAccel * this.t6 * this.t6;
            double v6 = v5 - maxAccel * this.t6;
            
            jerk = maxJerk;
            acceleration = -maxAccel + maxJerk * dt;
            velocity = v6 - maxAccel * dt + 0.5 * maxJerk * dt * dt;
            position = startPos + direction * 
                      (d1 + d2 + d3 + d4 + d5 + d6 + v6 * dt - 0.5 * maxAccel * dt * dt + (1.0 / 6.0) * maxJerk * dt * dt * dt);
        }
        
        else {
            return new SparkMaxMotionState(endPos, 0, 0, 0);
        }
        
        return new SparkMaxMotionState(
            position,
            direction * velocity,
            direction * acceleration,
            direction * jerk
        );
    }

    /**
     * Calcula el estado del perfil de movimiento en el tiempo actual
     * @param setPoint Posición objetivo
     * @param currentSetpoint Posición inicial (típicamente la posición actual del encoder)
     * @param SprofileOrTrapezoidal true = S-Curve, false = Trapezoidal
     * @param isPositionControl true = control de posición, false = control de velocidad
     * @return El voltaje calculado para aplicar al motor
     */
    public double CalculateProfileMotion(double setPoint, double currentSetpoint, boolean SprofileOrTrapezoidal, boolean isPositionControl) {
        
        // Si no hay perfil activo o cambió el objetivo, inicializar nuevo perfil
        if (!this.profileActive || Math.abs(this.profileTargetPosition - setPoint) > 0.001) {
            initializeProfile(currentSetpoint, setPoint, SprofileOrTrapezoidal);
        }
        
        // Calcular tiempo transcurrido
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - this.profileStartTime;
        
        // Obtener estado deseado del perfil
        SparkMaxMotionState desiredState;
        if (this.useJerkControl) {
            desiredState = getSCurveState(elapsedTime, this.profileStartPosition, this.profileTargetPosition);
        } else {
            desiredState = getTrapezoidalState(elapsedTime, this.profileStartPosition, this.profileTargetPosition);
        }
        
        // Guardar estado actual del perfil
        this.currentProfilePosition = desiredState.position;
        this.currentProfileVelocity = desiredState.velocity;
        this.currentProfileAcceleration = desiredState.acceleration;
        this.currentProfileJerk = desiredState.jerk;
        
        // Calcular feedforward
        double feedforward = this.calculateFeedForward(
            desiredState.velocity,
            desiredState.acceleration
        );
        
        // Calcular feedback (PID)
        double feedback;
        if (isPositionControl) {
            double currentPosition = this.getRelativeEncoderPosition();
            feedback = this.pidControllerVoltage.calculate(currentPosition, desiredState.position);
        } else {
            double currentVelocity = this.getRelativeEncoderVelocity();
            feedback = this.pidControllerVoltage.calculate(currentVelocity, desiredState.velocity);
        }
        
        // Combinar feedforward y feedback
        double output = feedforward + feedback;
        
        // Verificar si el perfil terminó
        if (elapsedTime >= this.totalProfileTime) {
            this.profileActive = false;
        }
        
        return MathUtil.clamp(output, -MAXIMUM_BATTERY_VOLTAGE, MAXIMUM_BATTERY_VOLTAGE);
    }
    
    /**
     * Inicializa un nuevo perfil de movimiento
     */
    private void initializeProfile(double startPos, double endPos, boolean useSCurve) {
        this.profileStartPosition = startPos;
        this.profileTargetPosition = endPos;
        this.profileStartTime = System.currentTimeMillis() / 1000.0;
        this.profileActive = true;
        this.useJerkControl = useSCurve;
        
        // Resetear PID para evitar acumulación de integral
        this.resetPid();
        
        // Calcular el perfil
        if (useSCurve) {
            calculateSCurveProfile(startPos, endPos);
        } else {
            calculateTrapezoidalProfile(startPos, endPos);
        }
    }

    /**
     * Control de posición con perfil de movimiento trapezoidal
     * @param desiredPosition Posición objetivo
     */
    public void MagicMotionPositionControl(double desiredPosition) {
        MagicMotionPositionControl(desiredPosition, false);
    }
    
    /**
     * Control de posición con perfil de movimiento
     * @param desiredPosition Posición objetivo
     * @param enableJerk true para usar S-Curve, false para trapezoidal
     */
    public void MagicMotionPositionControl(double desiredPosition, boolean enableJerk) {
        double currentPosition = this.getRelativeEncoderPosition();
        double voltage = CalculateProfileMotion(desiredPosition, currentPosition, enableJerk, true);
        this.setVoltage(voltage);
    }

    /**
     * Control de velocidad con perfil de movimiento trapezoidal
     * @param desiredVelocity Velocidad objetivo
     */
    public void MagicMotionVelocityControl(double desiredVelocity) {
        MagicMotionVelocityControl(desiredVelocity, false);
    }

    /**
     * Control de velocidad con perfil de movimiento
     * @param desiredVelocity Velocidad objetivo
     * @param enableJerk true para usar S-Curve, false para trapezoidal
     */
    public void MagicMotionVelocityControl(double desiredVelocity, boolean enableJerk) {
        double currentVelocity = this.getRelativeEncoderVelocity();
        double voltage = CalculateProfileMotion(desiredVelocity, currentVelocity, enableJerk, false);
        this.setVoltage(voltage);
    }

    // ============ UTILITY METHODS ============
    
    /**
     * Verifica si el perfil de movimiento ha terminado
     */
    public boolean isProfileFinished() {
        return !this.profileActive;
    }
    
    /**
     * Obtiene la posición actual del perfil (no del encoder)
     */
    public double getCurrentProfilePosition() {
        return this.currentProfilePosition;
    }
    
    /**
     * Obtiene la velocidad actual del perfil (no del encoder)
     */
    public double getCurrentProfileVelocity() {
        return this.currentProfileVelocity;
    }
    
    /**
     * Obtiene la aceleración actual del perfil
     */
    public double getCurrentProfileAcceleration() {
        return this.currentProfileAcceleration;
    }
    
    /**
     * Obtiene el jerk actual del perfil (solo para S-Curve)
     */
    public double getCurrentProfileJerk() {
        return this.currentProfileJerk;
    }
    
    /**
     * Cancela el perfil de movimiento actual
     */
    public void cancelProfile() {
        this.profileActive = false;
        this.stop();
    }
    
    /**
     * Obtiene el tiempo total estimado del perfil actual
     */
    public double getProfileTotalTime() {
        return this.totalProfileTime;
    }
    
    /**
     * Obtiene el tiempo transcurrido del perfil actual
     */
    public double getProfileElapsedTime() {
        if (!this.profileActive) {
            return 0;
        }
        double currentTime = System.currentTimeMillis() / 1000.0;
        return currentTime - this.profileStartTime;
    }
    
    /**
     * Verifica si hay un perfil activo
     */
    public boolean isProfileActive() {
        return this.profileActive;
    }

    // ============ DEPRECATED METHODS (mantener por compatibilidad) ============
    
    /**
     * @deprecated Usar CalculateProfileMotion en su lugar
     */
    @Deprecated
    private double stepForPosition(SparkMaxMotionState motionState, boolean enableJerk, boolean isPositiveDirection) {
        if (userConfiguration.dt <= 0) {
            return 0;
        }

        if (enableJerk) {
            if (userConfiguration.TargetJerk > 0) {
                double jerk = userConfiguration.TargetJerk * (isPositiveDirection ? 1 : -1);
                double acceleration = motionState.acceleration + jerk * userConfiguration.dt;
                acceleration = MathUtil.clamp(acceleration, -userConfiguration.TargetAcceleration, userConfiguration.TargetAcceleration);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                velocity = MathUtil.clamp(velocity, -userConfiguration.CruiseVelocity, userConfiguration.CruiseVelocity);
                double position = motionState.position + velocity * userConfiguration.dt;
                return position;
            } else {
                return 0;
            }
        } else {
            if (userConfiguration.TargetAcceleration > 0) {
                double acceleration = userConfiguration.TargetAcceleration * (isPositiveDirection ? 1 : -1);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                velocity = MathUtil.clamp(velocity, -userConfiguration.CruiseVelocity, userConfiguration.CruiseVelocity);
                double position = motionState.position + velocity * userConfiguration.dt;
                return position;
            } else {
                return 0;
            }
        }
    }

    /**
     * @deprecated Usar CalculateProfileMotion en su lugar
     */
    @Deprecated
    private double stepForVelocity(SparkMaxMotionState motionState, boolean enableJerk, boolean isPositiveDirection) {
        if (userConfiguration.dt <= 0) {
            return 0;
        }

        if (enableJerk) {
            if (userConfiguration.TargetJerk > 0) {
                double jerk = userConfiguration.TargetJerk * (isPositiveDirection ? 1 : -1);
                double acceleration = motionState.acceleration + jerk * userConfiguration.dt;
                acceleration = MathUtil.clamp(acceleration, -userConfiguration.TargetAcceleration, userConfiguration.TargetAcceleration);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                return velocity;
            } else {
                return 0;
            }
        } else {
            if (userConfiguration.TargetAcceleration > 0) {
                double acceleration = userConfiguration.TargetAcceleration * (isPositiveDirection ? 1 : -1);
                double velocity = motionState.velocity + acceleration * userConfiguration.dt;
                return velocity;
            } else {
                return 0;
            }
        }
    }
}