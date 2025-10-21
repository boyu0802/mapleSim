package frc.robot.Subsystem.Hanger;

import org.littletonrobotics.junction.AutoLog;

public interface HangerIO {

    @AutoLog
    public class HangerIOInputs{
        public HangerData hangerData = new HangerData(0,0,0);
    }

    record HangerData(
        double positionMeters,
        double motorVoltage,
        double velocityMetersPerSecond
    ){}

    default void updateInputs(HangerIOInputs inputs){}

    default void runPosition(double positionRotation){}

    default void update(){}

}
