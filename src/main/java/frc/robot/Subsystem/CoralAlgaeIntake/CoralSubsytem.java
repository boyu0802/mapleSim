package frc.robot.Subsystem.CoralAlgaeIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsytem extends SubsystemBase{
    private CoralIO io;
    private CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    public CoralSubsytem(CoralIO io){
        this.io = io;
    }

    public Command toIntake(){
        return run(() -> io.runPosition(0.1));
    }

    public Command toDefault(){
        return run(()->io.runPosition(0));
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        Logger.processInputs("Subsystem/Coral", inputs);

        if(RobotBase.isSimulation()) io.update();
    }}
