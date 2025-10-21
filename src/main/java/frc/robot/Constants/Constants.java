package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Robot;

public class Constants {

    public static enum states{
        Net (1.28,0.07,0.0,0.0),
        Coral_L1 (0.1,0.27,0.0,0.0),
        Coral_L2(0.25,0.375,0.0,0.0),
        Coral_L3(0.65,0.375,0.0,0.0),
        Coral_L4(1.28,0.375,0.0,0.0),
        Hang_Prep(0.03,0.0,0.25,0.48),
        Hang(0.03,0.0,0.0,0.48),
        Coral_Intake(0.4,0.125,0.0,0.0),
        Normal(0.01,0.0,0.0,0.01),
        Algae_Intake(0.0,0.0,0.0,0.48),
        Algae_L2(0.33,0.375,0.0,0.0),
        Algae_L3(0.68,0.375,0.0,0.0);

        public double elevatorPosition;
        public double coralPosition;
        public double hangerPosition;
        public double algaePosition;

        private states(double elevatorPosition,double coralPosition,double hangerPosition, double algaePosition) {
            this.elevatorPosition = elevatorPosition;
            this.coralPosition = coralPosition;
            this.hangerPosition = hangerPosition;
            this.algaePosition = algaePosition;
        }
    }


    public enum robotStates{
        Net,
        Coral_L1,
        Coral_L2,
        Coral_L3,
        Coral_L4,
        Hang_Prep,
        Hang,
        Coral_Intake,
        Normal,
        Algae_Intake,
        Algae_L2,
        Algae_L3;
        
    }
    

    public static final class AlgaeIntakeConstants{
        public static final double PIVOT_MOTOR_GEAR_RATIO = 100.0;
        public static final double ROLLER_GEAR_RATIO = 3.0;

        public static final double PIVOT_MOMENT_OF_INERTIA = 0.002701; 

        public static final SparkFlexConfig PIVOT_MOTOR_CONFIG = new SparkFlexConfig();
        public static final SparkFlexConfig ROLLER_MOTOR_CONFIG = new SparkFlexConfig();


        public static final double PIVOT_KS = 0.0;
        public static final double PIVOT_KV = 0.0;
        public static final double PIVOT_KA = 0.0;
        public static final double PIVOT_KG = 0.0;


        public static final double ROLLER_INTAKE_VELOCITY = -15.0;
        public static final double ROLLER_REVERSE_VELOCITY = 20.0;

        public static final ArmFeedforward PIVOT_FEED_FORWARD = new ArmFeedforward(PIVOT_KS, PIVOT_KG, PIVOT_KV,PIVOT_KA);


        static{
            PIVOT_MOTOR_CONFIG.inverted(true);

            PIVOT_MOTOR_CONFIG.encoder.positionConversionFactor(1/PIVOT_MOTOR_GEAR_RATIO);
            PIVOT_MOTOR_CONFIG.encoder.velocityConversionFactor((1/PIVOT_MOTOR_GEAR_RATIO)/60.0);

            PIVOT_MOTOR_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12).inverted(false);
            

            if(Robot.isSimulation()){

                PIVOT_MOTOR_CONFIG.absoluteEncoder
                    .zeroOffset(0);

                PIVOT_MOTOR_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(9, 0, 0.0)
                    .iZone(0)
                    .outputRange(-1,1);

            }else{
                PIVOT_MOTOR_CONFIG.absoluteEncoder
                    .zeroOffset(0); //tune real world


                PIVOT_MOTOR_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.02, 0, 0.0) //tune real world
                    .iZone(0)
                    .outputRange(-1,1);
            }
        }

    }

    public static final class ElevatorConstants{
        public static final double ELEVATOR_GEAR_RATIO = 9.0;

        public static final double ELEVATOR_MIN_LENGTH = 0.0; //TODO: guess
        public static final double ELEVATOR_MAX_LENGTH = 1.2; //TODO : guess

        public static final double ELEVATOR_CARRIAGE_MASS = 3.0; //TODO: guess
        public static final double ELEVATOR_DRUM_RADIUS = 0.05;

        public static final double ELEVAROR_MAX_VELOCITY = 150.0; 
        public static final double ELEVAROR_MAX_ACCEL = 100.0; 
        public static final double ELEVATOR_MAX_JERK = 300.0; 

        public static final double ELEVATOR_SIM_KS = 0.0;
        public static final double ELEVATOR_SIM_KV = 0.762;
        public static final double ELEVATOR_SIM_KA = 0.0;
        public static final double ELEVATOR_SIM_KG = 0.762;
        public static final double ELEVATOR_SIM_KP = 20.0;
        public static final double ELEVATOR_SIM_KI = 0.0;
        public static final double ELEVATOR_SIM_KD = 0.0;

        public static final double ELEVATOR_REAL_KS = 0.0;
        public static final double ELEVATOR_REAL_KV = 0.0;
        public static final double ELEVATOR_REAL_KA = 0.0;
        public static final double ELEVATOR_REAL_KG = 0.0;
        public static final double ELEVATOR_REAL_KP = 0.0;
        public static final double ELEVATOR_REAL_KI = 0.0;
        public static final double ELEVATOR_REAL_KD = 0.0;

        public static final double ROTOR_TO_METERS =  2.0 * Math.PI * ELEVATOR_DRUM_RADIUS;

        public static final TalonFXConfiguration ELEVATOR_SIM_CONFIG_L = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(120))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO/ROTOR_TO_METERS))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                    // .withContinuousWrap(false))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(ELEVATOR_SIM_KP)
                    .withKI(ELEVATOR_SIM_KI)
                    .withKD(ELEVATOR_SIM_KD)
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKG(ELEVATOR_SIM_KG)
                    .withKS(ELEVATOR_SIM_KS)
                    .withKV(ELEVATOR_SIM_KV)
                    .withKA(ELEVATOR_SIM_KA))

                .withMotionMagic(
                    new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVAROR_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ELEVAROR_MAX_ACCEL)
                    .withMotionMagicJerk(ELEVATOR_MAX_JERK)
                    )
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH)
                );
            

        public static final TalonFXConfiguration ELEVATOR_SIM_CONFIG_R = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(120))
            .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(12)
                        .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
                        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                        // .withContinuousWrap(false))
            .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
    
            .withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_SIM_KP)
                        .withKI(ELEVATOR_SIM_KI)
                        .withKD(ELEVATOR_SIM_KD)
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKG(ELEVATOR_SIM_KG)
                        .withKS(ELEVATOR_SIM_KS)
                        .withKV(ELEVATOR_SIM_KV)
                        .withKA(ELEVATOR_SIM_KA))
    
            .withMotionMagic(
                        new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(ELEVAROR_MAX_VELOCITY)
                        .withMotionMagicAcceleration(ELEVAROR_MAX_ACCEL)
                        .withMotionMagicJerk(ELEVATOR_MAX_JERK)
                        )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH)
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH)
                    );
            
        

    }


    public static final class HangerConstants{
        public static final double HANG_GEAR_RATIO = 45.0;
        public static final double HANG_LENGTH = 0.22;

        public static final double HANG_SIM_KP = 22.0;
        public static final double HANG_SIM_KI = 0.0;
        public static final double HANG_SIM_KD = 0.0;
        public static final double HANG_SIM_KS = 0.0;
        public static final double HANG_SIM_KV = 0.80;
        public static final double HANG_SIM_KA = 0.0;
        public static final double HANG_SIM_KG = 0.010;


        public static final CANcoderConfiguration HANG_ENCODER_CONFIG = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withMagnetOffset(0.0)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        public static final TalonFXConfiguration HANG_SIM_CONFIG = new TalonFXConfiguration()

            // .withFeedback(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder).withFeedbackRemoteSensorID(15))
                       
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(HANG_SIM_KP)
                    .withKI(HANG_SIM_KI)
                    .withKD(HANG_SIM_KD)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(HANG_SIM_KS)
                    .withKV(HANG_SIM_KV)
                    .withKA(HANG_SIM_KA)
                    .withKG(HANG_SIM_KG))

            
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(20.0)
                    .withMotionMagicAcceleration(40.0));
            
            
    }

    public static final class coralWristConstants{

        public static final double CORAL_WRIST_LENGTH = 34.176;
        public static final double CORAL_WRIST_GEAR_RATIO = 225.0/4.0;


        public static final double CORAL_WRIST_KP = 22;
        public static final double CORAL_WRIST_KI = 0;
        public static final double CORAL_WRIST_KD = 0.0;
        public static final double CORAL_WRIST_KS = 0.0;
        public static final double CORAL_WRIST_KV = 0.67;
        public static final double CORAL_WRIST_KA = 0.0;
        public static final double CORAL_WRIST_KG = 0.5;
        
        

        public static final double CORAL_WRIST_MAX_VELOCITY = 320;
        public static final double CORAL_WRIST_MAX_ACCEL = 100.0;
        public static final double CORAL_WRIST_MAX_JERK = 500.0;



        public static final TalonFXConfiguration CORAL_WRISTCONFIG = new TalonFXConfiguration()

            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    //.withFeedbackRemoteSensorID(CORAL_WRIST_ID.getDeviceNumber())
                    //.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(CORAL_WRIST_GEAR_RATIO))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                    
                    //.withContinuousWrap(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(CORAL_WRIST_KP)
                    .withKI(CORAL_WRIST_KI)
                    .withKD(CORAL_WRIST_KD)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(CORAL_WRIST_KS)
                    .withKV(CORAL_WRIST_KV)
                    .withKA(CORAL_WRIST_KA)
                    .withKG(CORAL_WRIST_KG))

            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CORAL_WRIST_MAX_VELOCITY)
                    .withMotionMagicAcceleration(CORAL_WRIST_MAX_ACCEL)
                    .withMotionMagicJerk(CORAL_WRIST_MAX_JERK)
                    );
    }

}
    