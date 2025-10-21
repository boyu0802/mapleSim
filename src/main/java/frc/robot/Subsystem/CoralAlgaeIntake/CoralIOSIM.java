package frc.robot.Subsystem.CoralAlgaeIntake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotState;
import frc.robot.Constants.Constants;

public class CoralIOSIM extends CoralIOCTRE{
    private final TalonFXSimState coralWristSimState;
    private final DCMotor coralWristMotorSim = DCMotor.getKrakenX60(1);

    private final SingleJointedArmSim coralWristSim = new SingleJointedArmSim(
        coralWristMotorSim, 
        Constants.coralWristConstants.CORAL_WRIST_GEAR_RATIO, 
        0.02856,
        Constants.coralWristConstants.CORAL_WRIST_LENGTH, 
        Units.degreesToRadians(-120),
        Units.degreesToRadians(0),
        false,
        Units.degreesToRadians(0));

    public CoralIOSIM(TalonFXConfiguration coralConfig){
        super(coralConfig);
        coralWristSimState = new TalonFXSimState(getcoralWristMotor());
    }


    private Pose3d coralWristPose = new Pose3d(0.325,0.0,0.684,new Rotation3d());


    public void update(){
        super.update();
        coralWristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        Logger.recordOutput("coralWrist/motorVoltage",coralWristSimState.getMotorVoltage());
        coralWristSim.setInput(coralWristSimState.getMotorVoltage());

        
        // Next, we update it. The standard loop time is 20ms.
        coralWristSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        coralWristSimState.setRawRotorPosition(Units.radiansToRotations(coralWristSim.getAngleRads()));
        coralWristSimState.setRotorVelocity(Units.radiansToRotations(coralWristSim.getVelocityRadPerSec()));
        //coralWristEncoderSimState.setRawPosition(Units.radiansToRotations(coralWristSim.getAngleRads()));

        coralWristPose = new Pose3d(0.325,0.0,0.684 + RobotState.getElevatorPosition(), new Rotation3d(0,-coralWristSim.getAngleRads(),0));

        Logger.recordOutput("coral/coralWristAngle",Units.radiansToDegrees(coralWristSim.getAngleRads()));

        Logger.recordOutput("coral/coralWristPose",coralWristPose);
    }
}
