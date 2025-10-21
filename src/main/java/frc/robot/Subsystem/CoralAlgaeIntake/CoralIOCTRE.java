package frc.robot.Subsystem.CoralAlgaeIntake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;


public class CoralIOCTRE implements CoralIO {
    private final TalonFX coralWristMotor = new TalonFX(12);
    // private final CANcoder coralEncoder = new CANcoder(15);
    private StatusSignal<Angle> position = coralWristMotor.getPosition();
    private StatusSignal<AngularVelocity> velocity = coralWristMotor.getVelocity();
    private StatusSignal<Voltage> voltage = coralWristMotor.getMotorVoltage();

    private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);


    public CoralIOCTRE(TalonFXConfiguration coralConfig){
        coralWristMotor.getConfigurator().apply(coralConfig);
        coralWristMotor.setPosition(0);
        // coralEncoder.getConfigurator().apply(canCoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage);
    }

    public void updateInputs(CoralIOInputs inputs){

        BaseStatusSignal.refreshAll(position,velocity,voltage);

        inputs.coralData = new CoralData(
            position.getValueAsDouble(),
            voltage.getValueAsDouble(),
            velocity.getValueAsDouble()
        );
    }

    public void runPosition(double positionRotation){
        coralWristMotor.setControl(motionMagicPosition.withPosition(positionRotation).withSlot(0));
    }

    public TalonFX getcoralWristMotor(){
        return coralWristMotor;
    }

    // public CANcoder getcoralEncoder(){
    //     return coralEncoder;
    // }

    public void update(){
        Logger.recordOutput("coral/setpoint",coralWristMotor.getClosedLoopReference().getValueAsDouble());
        
    }
}
