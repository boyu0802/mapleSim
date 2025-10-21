package frc.robot.Subsystem.Hanger;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Constants;

public class HangerIOSIM extends HangerIOCTRE{
    private final TalonFXSimState hangerSimState;
    // private final CANcoderSimState hangerEncoderSimState;
    private final DCMotor hangerMotorSim = DCMotor.getKrakenX60(1);
    
    private final SingleJointedArmSim hangerSim = new SingleJointedArmSim(
        hangerMotorSim, 
        Constants.HangerConstants.HANG_GEAR_RATIO, 
        0.02856,
        Constants.HangerConstants.HANG_LENGTH, 
        Units.degreesToRadians(0),
        Units.degreesToRadians(90),
        false,
        0);

    private Pose3d hangerPose3d = new Pose3d(0.0,0.34,0.3,new Rotation3d());

    public HangerIOSIM(TalonFXConfiguration hangConfig, CANcoderConfiguration canCoderConfig){
        super(hangConfig, canCoderConfig);
        hangerSimState = new TalonFXSimState(getHangerMotor());
        // hangerEncoderSimState = new CANcoderSimState(getHangerEncoder());
    }

    public void update(){
        super.update();
        hangerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        Logger.recordOutput("hanger/motorVoltage",hangerSimState.getMotorVoltage());
        hangerSim.setInput(hangerSimState.getMotorVoltage());

        
        // Next, we update it. The standard loop time is 20ms.
        hangerSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        hangerSimState.setRawRotorPosition(Units.radiansToRotations(hangerSim.getAngleRads()));
        hangerSimState.setRotorVelocity(Units.radiansToRotations(hangerSim.getVelocityRadPerSec()));
        // hangerEncoderSimState.setRawPosition(Units.radiansToRotations(hangerSim.getAngleRads()));

        hangerPose3d = new Pose3d(0.0,0.34,0.3, new Rotation3d(-hangerSim.getAngleRads(),0,0));


        Logger.recordOutput("Hanger/hangerPose", hangerPose3d);
    }
}
