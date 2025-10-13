package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Constants {
    

    public static final class AlgaeIntakeConstants{
        public static final double PIVOT_MOTOR_GEAR_RATIO = 1.0/100.0;
        public static final double ROLLER_GEAR_RATIO = 1.0 / 3.0;

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

            PIVOT_MOTOR_CONFIG.encoder.positionConversionFactor(PIVOT_MOTOR_GEAR_RATIO);
            PIVOT_MOTOR_CONFIG.encoder.velocityConversionFactor(PIVOT_MOTOR_GEAR_RATIO/60.0);

            PIVOT_MOTOR_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12).inverted(false);
            
            PIVOT_MOTOR_CONFIG.absoluteEncoder
                .zeroOffset(0);

            PIVOT_MOTOR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.0, 0, 0.0)
                .iZone(0)
                .outputRange(-1,1);
        }

    }


}
    