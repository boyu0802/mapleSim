package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {
        public ElevatorData elevatorData = new ElevatorData(0,0,0);
    }

    record ElevatorData(
        double positionMeters,
        double motorVoltage,
        double velocityMetersPerSecond
    ){}

    default void updateInputs(ElevatorIOInputs inputs){}

    default void runPosition(double positionMeters){}

    default void runVoltage(double volts){}

    default double getElevatorLeftPosition(){ return 0.0; }

    default void update(){}
}
