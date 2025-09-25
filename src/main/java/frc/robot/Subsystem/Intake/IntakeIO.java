package frc.robot.Subsystem.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {


    public default void setDeployed(boolean deployed) {}

    public default boolean hasGamePiece() {return false;}

    
} 
