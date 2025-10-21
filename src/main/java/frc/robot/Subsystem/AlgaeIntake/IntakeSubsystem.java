package frc.robot.Subsystem.AlgaeIntake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.SubsystemDataProcessor;

public class IntakeSubsystem extends SubsystemBase{
    

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private SysIdRoutine IntakePivotRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("IntakeSubsystem/SysID/state", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (voltage) -> io.runPivotVoltage(voltage.in(Volts)),
            null, 
            this));

    public IntakeSubsystem(IntakeIO io){
        this.io  = io;
    }



    public Command toDefaultPosition(){
        return Commands.run(()-> io.runPivotPosition(0.012),this);
    }

    public Command toIntakePosition(){
        return Commands.run(()-> io.runPivotPosition(0.488),this);

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return IntakePivotRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return IntakePivotRoutine.dynamic(direction);
    }

    public Command stop(){
        return Commands.runOnce(()->{
            io.runPivotVoltage(0);
            io.runRollerVelocity(0);
        });
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        Logger.processInputs("Subsystem/Intake", inputs);

        if(RobotBase.isSimulation()) io.update();
    }

}
