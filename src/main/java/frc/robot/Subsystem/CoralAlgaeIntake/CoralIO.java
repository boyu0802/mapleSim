package frc.robot.Subsystem.CoralAlgaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public class CoralIOInputs{
        public CoralData coralData = new CoralData(0,0,0);
    }

    record CoralData(
        double positionRotations,
        double motorVoltage,
        double velocityRotationPerSecond
    ){}

    default void updateInputs(CoralIOInputs inputs){}

    default void runPosition(double positionRotation){}

    default void update(){}
    

    
}
