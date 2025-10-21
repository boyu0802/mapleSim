package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class ElevatorSubsystem extends SubsystemBase{
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    public Command toL1(){
        return run(() -> io.runPosition(1.2));
    }

    public Command toZero(){
        return run(() -> io.runPosition(0.0));
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        Logger.processInputs("Subsystem/Elevator", inputs);

        if(RobotBase.isSimulation()) io.update();

        RobotState.setElevatorPosition(io.getElevatorLeftPosition());
    }
}
