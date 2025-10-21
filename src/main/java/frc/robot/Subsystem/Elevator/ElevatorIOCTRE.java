package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Constants;

public class ElevatorIOCTRE implements ElevatorIO{
    private TalonFX elevatorLeft = new TalonFX(9);
    private TalonFX elevatorRight = new TalonFX(10);
    private StatusSignal<Angle> position = elevatorLeft.getPosition();
    private StatusSignal<AngularVelocity> velocity = elevatorLeft.getVelocity();
    private StatusSignal<Voltage> voltage = elevatorLeft.getMotorVoltage();

    private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);

    public ElevatorIOCTRE(TalonFXConfiguration leftConfig, TalonFXConfiguration rightConfig){
        elevatorLeft.getConfigurator().apply(leftConfig);
        elevatorRight.getConfigurator().apply(rightConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage);
    }

    public void updateInputs(ElevatorIOInputs inputs){

        BaseStatusSignal.refreshAll(position,velocity,voltage);

        inputs.elevatorData = new ElevatorData(
            position.getValueAsDouble(),
            voltage.getValueAsDouble(),
            velocity.getValueAsDouble()
        );
    }

    public void runPosition(double positionMeters){
        elevatorRight.setControl(motionMagicPosition.withPosition(positionMeters).withSlot(0));
        elevatorLeft.setControl(motionMagicPosition.withPosition(positionMeters).withSlot(0));
    }

    public TalonFX getElevatorRight(){
        return elevatorRight;
    }

    public TalonFX getElevatorLeft(){
        return elevatorLeft;
    }

    public double getElevatorLeftPosition(){
        return elevatorLeft.getPosition().getValueAsDouble();
    }
    
    public void update(){
        Logger.recordOutput("Elevator/setpoint",elevatorLeft.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Elevator/closedLoopError", elevatorLeft.getClosedLoopError().getValueAsDouble());
    }

}
