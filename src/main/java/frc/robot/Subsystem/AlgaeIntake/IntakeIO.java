package frc.robot.Subsystem.AlgaeIntake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs{
        public IntakePivotData pivotData = new IntakePivotData(Rotation2d.kZero, 0, 0, 0);
        public IntakeRollerData rollerData = new IntakeRollerData( 0, 0, 0);

    }

    record IntakePivotData(
        Rotation2d absoluteAngle,
        double positionRotations,
        double motorVoltage,
        double velocityRotationsPerSecond
    ){}
    
    record IntakeRollerData(
        double positionRotations,
        double motorVoltage,
        double velocityRotationsPerSecond   
    ){}

    default void updateInputs(IntakeIOInputs inputs){}

    default void runPivotPosition(double positionRotations){}

    default void runRollerVelocity(double velocity){}

    default void setPivotPID(double p, double i, double d){}

    default void setRollerPID(double p, double i, double d){}

    default void runPivotVoltage(double volts){}

    default void update(){}

} 
