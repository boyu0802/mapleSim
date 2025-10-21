package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState {
    public static double elvatorPositionMeters = 0.0;
    public static double elvatorVelocityMetersPerSecond = 0.0;

    public static double hangerPositionRotations = 0.0;
    public static double hangerVelocityRotationsPerSecond = 0.0;

    public static double algaePositionRotations = 0.0;
    public static double algaeVelocityRotationsPerSecond = 0.0;

    public static Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    public static RobotState instance = null;

    public RobotState(){}

    public static RobotState getInstance(){
        if(instance == null){
            instance = new RobotState();
        }
        return instance;
    }

    public static void setElevatorPosition(double positionMeters){
        elvatorPositionMeters = positionMeters;
    }

    public static double getElevatorPosition(){
        return elvatorPositionMeters;
    }
}
