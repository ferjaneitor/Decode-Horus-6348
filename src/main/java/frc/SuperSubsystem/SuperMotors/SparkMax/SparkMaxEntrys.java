package frc.SuperSubsystem.SuperMotors.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class SparkMaxEntrys {

    public static final class SuperSparkMaxConfig {

        public double  Kg, Ks, Kv, Ka, kGcos, kGcosRatio, kp, ki, kd, kError, dt, CruiseVelocity, TargetAcceleration, TargetJerk,kCurrentChop, kRampRate, kSoftLimitFwd, kSoftLimitRev, kPositionConvertionFactor, kVelocityConversionFactor;

        public boolean KLimitFwdPolarity, kLimitRevPolarity,kIsInverted, kIsBrakeMode, kIsEncoderInverted, kSoftLimitFwdEnabled, kSoftLimitRevEnabled, enableKGCos;

        public int SmartCurrentFreeLimit, SmartCurrentConfig, SmartCurrentStallLimit;

        

        public SuperSparkMaxConfig(){
                //Gratitacional Constant
            this.Kg = 0.0;
                //Static Friction Constant
            this.Ks = 0.0;
                //Velocity Constant
            this.Kv = 0.0;
                //Acceleration Constant
            this.Ka = 0.0;
                //Gravitational Cosine Constant(For arms and pivot Points)
            this.kGcos = 0.0;
                //Gravitational Cosine Ratio(For arms and pivot Points)
            this.kGcosRatio = 1.0;
                //Proportional Constant
            this.kp = 0.0;
                //Integral Constant
            this.ki = 0.0;
                //Derivative Constant
            this.kd = 0.0;
                //Allowable Closed Loop Error
            this.kError = 0.0;
                //Delta Time for Motion Magic Calculations
            this.dt = 0.02;
                //Motion Magic Cruise Velocity
            this.CruiseVelocity = 0.0;
                //Motion Magic Target Acceleration
            this.TargetAcceleration = 0.0;
                //Motion Magic Target Jerk
            this.TargetJerk = 0.0;

                //Current Chop
            this.kCurrentChop = 115;

                //Ramp Rate
            this.kRampRate = 0.0f;
                //Soft Limit Forward
            this.kSoftLimitFwd = 0.0f;
                //Soft Limit Reverse
            this.kSoftLimitRev = 0.0f;
                //Position Convertion Factor
            this.kPositionConvertionFactor = 1.0f;
                //Velocity Conversion Factor
            this.kVelocityConversionFactor = 1.0f;

                //Limit Forward Polarity
            this.KLimitFwdPolarity = true;
                //Limit Reverse Polarity
            this.kLimitRevPolarity = true;
                //Is Inverted
            this.kIsInverted = false;
                //Is Brake Mode
            this.kIsBrakeMode = true;

            this.kSoftLimitFwdEnabled = false;
            this.kSoftLimitRevEnabled = false;

                //Smart Current Free Limit
            this.SmartCurrentFreeLimit = 20;
                //Smart Current Config
            this.SmartCurrentConfig = 10000;

            this.SmartCurrentStallLimit = 80;

        }

        public void Kg(double Kg){this.Kg = Kg;}
        public double Kg(){return this.Kg;}

        public void Ks(double Ks){this.Ks = Ks;}
        public double Ks(){return this.Ks;}

        public void Kv(double Kv){this.Kv = Kv;}
        public double Kv(){return this.Kv;}

        public void Ka(double Ka){this.Ka = Ka;}
        public double Ka(){return this.Ka;}

        public void kGcos(double MotorRotation){
            if(this.enableKGCos){
                    this.kGcos = Units.rotationsToRadians(MathUtil.clamp(MotorRotation, - this.kPositionConvertionFactor, this.kPositionConvertionFactor));
            }
        }
        public double kGcos(){return this.kGcos;}

        public void kGcosRatio(double kGcosRatio){this.kGcosRatio = kGcosRatio;}
        public double kGcosRatio(){return this.kGcosRatio;}

        public void kp(double kp){this.kp = kp;}
        public double kp(){return this.kp;}

        public void ki(double ki){this.ki = ki;}
        public double ki(){return this.ki;}

        public void kd(double kd){this.kd = kd;}
        public double kd(){return this.kd;}

        public void kError(double kError){this.kError = kError;}
        public double kError(){return this.kError;}

        public void dt(double dt){this.dt = dt;}
        public double dt(){return this.dt;}

        public void CruiseVelocity(double CruiseVelocity){this.CruiseVelocity = CruiseVelocity;}
        public double CruiseVelocity(){return this.CruiseVelocity;}

        public void TargetAcceleration(double TargetAcceleration){this.TargetAcceleration = TargetAcceleration;}
        public double TargetAcceleration(){return this.TargetAcceleration;}

        public void TargetJerk(double TargetJerk){this.TargetJerk = TargetJerk;}
        public double TargetJerk(){return this.TargetJerk;}

        public void kCurrentChop(double kCurrentChop){this.kCurrentChop = kCurrentChop;}
        public double kCurrentChop(){return this.kCurrentChop;}

        public void kRampRate(float kRampRate){this.kRampRate = kRampRate;}
        public double kRampRate(){return this.kRampRate;}

        public void kSoftLimitFwd(float kSoftLimitFwd){this.kSoftLimitFwd = kSoftLimitFwd;}
        public double kSoftLimitFwd(){return this.kSoftLimitFwd;}

        public void kSoftLimitRev(float kSoftLimitRev){this.kSoftLimitRev = kSoftLimitRev;}
        public double kSoftLimitRev(){return this.kSoftLimitRev;}

        public void kPositionConvertionFactor(float kPositionConvertionFactor){this.kPositionConvertionFactor = kPositionConvertionFactor;}
        public double kPositionConvertionFactor(){return this.kPositionConvertionFactor;}

        public void kVelocityConversionFactor(float kVelocityConversionFactor){this.kVelocityConversionFactor = kVelocityConversionFactor;}
        public double kVelocityConversionFactor(){return this.kVelocityConversionFactor;}

        public void KLimitFwdPolarity(boolean KLimitFwdPolarity){this.KLimitFwdPolarity = KLimitFwdPolarity;}
        public boolean KLimitFwdPolarity(){return this.KLimitFwdPolarity;}

        public void kLimitRevPolarity(boolean kLimitRevPolarity){this.kLimitRevPolarity = kLimitRevPolarity;}
        public boolean kLimitRevPolarity(){return this.kLimitRevPolarity;}

        public void kIsInverted(boolean kIsInverted){this.kIsInverted = kIsInverted;}
        public boolean kIsInverted(){return this.kIsInverted;}

        public void kIsEncoderInverted(boolean kIsEncoderInverted){this.kIsEncoderInverted = kIsEncoderInverted;}
        public boolean kIsEncoderInverted(){return this.kIsEncoderInverted;}

        public void kIsBrakeMode(boolean kIsBrakeMode){this.kIsBrakeMode = kIsBrakeMode;}
        public boolean kIsBrakeMode(){return this.kIsBrakeMode;}

        public void SoftLimitFwdEnabled(boolean kSoftLimitFwdEnabled){this.kSoftLimitFwdEnabled = kSoftLimitFwdEnabled;}
        public boolean SoftLimitFwdEnabled(){return this.kSoftLimitFwdEnabled;}

        public void SoftLimitRevEnabled(boolean kSoftLimitRevEnabled){this.kSoftLimitRevEnabled = kSoftLimitRevEnabled;}
        public boolean SoftLimitRevEnabled(){return this.kSoftLimitRevEnabled;}

    }

    public static final class SparkMaxMotionState{
        public final double position;

        public final double velocity;

        public final double acceleration;

        public final double jerk;

        public SparkMaxMotionState(double position, double velocity, double acceleration, double jerk){
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.jerk = jerk;
        }
    }
    
}
