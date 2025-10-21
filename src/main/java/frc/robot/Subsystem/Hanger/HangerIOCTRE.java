package frc.robot.Subsystem.Hanger;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class HangerIOCTRE implements HangerIO {
    private final TalonFX hangerMotor = new TalonFX(11);
    // private final CANcoder hangerEncoder = new CANcoder(15);
    private StatusSignal<Angle> position = hangerMotor.getPosition();
    private StatusSignal<AngularVelocity> velocity = hangerMotor.getVelocity();
    private StatusSignal<Voltage> voltage = hangerMotor.getMotorVoltage();

    private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);


    public HangerIOCTRE(TalonFXConfiguration hangConfig,CANcoderConfiguration canCoderConfig){
        hangerMotor.getConfigurator().apply(hangConfig);
        hangerMotor.setPosition(0);
        // hangerEncoder.getConfigurator().apply(canCoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage);
    }

    public void updateInputs(HangerIOInputs inputs){

        BaseStatusSignal.refreshAll(position,velocity,voltage);

        inputs.hangerData = new HangerData(
            position.getValueAsDouble(),
            voltage.getValueAsDouble(),
            velocity.getValueAsDouble()
        );
    }

    public void runPosition(double positionRotation){
        hangerMotor.setControl(motionMagicPosition.withPosition(positionRotation).withSlot(0));
    }

    public TalonFX getHangerMotor(){
        return hangerMotor;
    }

    // public CANcoder getHangerEncoder(){
    //     return hangerEncoder;
    // }

    public void update(){
        Logger.recordOutput("Hanger/setpoint",hangerMotor.getClosedLoopReference().getValueAsDouble());
        
    }
}
