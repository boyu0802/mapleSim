package frc.robot.Subsystem.Hanger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangerSubsystem extends SubsystemBase {
    private HangerIO io;
    private HangerIOInputsAutoLogged inputs = new HangerIOInputsAutoLogged();

    public HangerSubsystem(HangerIO io){
        this.io = io;   
    }

    public Command toCage(){
        return run(() -> io.runPosition(0.25));
    }

    public Command hang(){
        return run(() -> io.runPosition(0));
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);

        Logger.processInputs("Subsystem/hanger", inputs);

        if(RobotBase.isSimulation()) io.update();
    }
}
