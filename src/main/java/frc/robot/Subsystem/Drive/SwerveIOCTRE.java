package frc.robot.Subsystem.Drive;

import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX,TalonFX,CANcoder> implements SwerveIO{
    private HashMap<String,BaseStatusSignal> frontLeftSignals = new HashMap<>();
    private HashMap<String,BaseStatusSignal> backLeftSignals = new HashMap<>();
    private HashMap<String,BaseStatusSignal> backRightSignals = new HashMap<>();
    private HashMap<String,BaseStatusSignal> frontRightSignals = new HashMap<>();

    public HashMap<Integer,HashMap<String,BaseStatusSignal>> swerveModuleSignals = new HashMap<>();
    
    @SafeVarargs
    public SwerveIOCTRE(SwerveDrivetrainConstants constants, SwerveModuleConstants<?,?,?>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        
        swerveModuleSignals.put(0,frontLeftSignals);
        swerveModuleSignals.put(1,backLeftSignals);
        swerveModuleSignals.put(2,backRightSignals);
        swerveModuleSignals.put(3,frontRightSignals);

        for(int i = 0; i < 4; i++){
            var driveMotor = this.getModule(i).getDriveMotor();
            var angleMotor = this.getModule(i).getSteerMotor();
            var canCoder = this.getModule(i).getEncoder();
            var moduleMap = swerveModuleSignals.get(i);

            moduleMap.put("drivePosition", driveMotor.getPosition());
            moduleMap.put("driveVelocity", driveMotor.getVelocity());
            moduleMap.put("driveCurrent", driveMotor.getSupplyCurrent());
            moduleMap.put("driveVoltage", driveMotor.getMotorVoltage());

            moduleMap.put("anglePosition", angleMotor.getPosition());
            moduleMap.put("angleVelocity", angleMotor.getVelocity());
            moduleMap.put("angleCurrent", angleMotor.getSupplyCurrent());
            moduleMap.put("angleVoltage", angleMotor.getMotorVoltage());
            moduleMap.put("angleAbsolutePosition", canCoder.getPosition());
        }
    }

    public void updateSwerveInputs(SwerveIOInputs inputs){
        inputs.gyroAngle = this.getStateCopy().RawHeading;
        inputs.moduleStates = this.getStateCopy().ModuleStates;
        inputs.modulePositions = this.getStateCopy().ModulePositions;
        inputs.timeStamp = this.getStateCopy().Timestamp;
        inputs.robotPose = this.getStateCopy().Pose;
        inputs.targetStates = this.getStateCopy().ModuleTargets;
        inputs.fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.getStateCopy().Speeds, this.getStateCopy().Pose.getRotation());
    }

    public void updateModuleInputs(ModuleIOInputs... inputs){

        for(int i = 0; i < 4; i++){
            var moduleMap = swerveModuleSignals.get(i);

            inputs[i].driveId = this.getModule(i).getDriveMotor().getDeviceID();
            inputs[i].drivePosition = moduleMap.get("drivePosition").getValueAsDouble();
            inputs[i].driveVelocity = moduleMap.get("driveVelocity").getValueAsDouble();
            inputs[i].driveAppliedVolts = moduleMap.get("driveVoltage").getValueAsDouble();
            inputs[i].steerId = this.getModule(i).getSteerMotor().getDeviceID();
            inputs[i].steerAbosolutePosition = Rotation2d.fromRotations(moduleMap.get("angleAbsolutePosition").getValueAsDouble());
            inputs[i].steerVelocity = moduleMap.get("angleVelocity").getValueAsDouble();
            inputs[i].steerAppliedVolts = moduleMap.get("angleVoltage").getValueAsDouble();
        }

    }


    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }
   
    @Override
    public void refreshData(){
        for(int i = 0; i < 4; i++){
            var moduleMap = swerveModuleSignals.get(i);
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[]{}));
        } 
    }

    public void periodic(){
        // SmartDashboard.putNumber("module 2 drive motor position", getModule(1).getDriveMotor().getPosition(true).getValueAsDouble());
        // SmartDashboard.putNumber("module 2 drive rotor position", getModule(1).getDriveMotor().getRotorPosition(true).getValueAsDouble());
        // SmartDashboard.putNumber("module 2 drive motor voltage", getModule(1).getDriveMotor().getSimState().getMotorVoltage());
        
        
        
        
        // SmartDashboard.putNumber("module 2 steer motor ", getModule(1).getSteerMotor().getPosition().getValueAsDouble()); 
        // SmartDashboard.putNumber("module 2 steer motor position", getModule(1).getSteerMotor().getPosition().getValue().magnitude());
        // SmartDashboard.putNumber("module 2 cancoder position", getModule(1).getEncoder().getPosition().getValue().magnitude());
        // SmartDashboard.putNumber("module 2 drive motor id", getModule(1).getDriveMotor().getDeviceID());
        // SmartDashboard.putNumber("module 2 steer motor id", getModule(1).getSteerMotor().getDeviceID());
        // SmartDashboard.putNumber("module 2 cancoder id", getModule(1).getEncoder().getDeviceID());
        

        // SmartDashboard.putNumber("module 2 drive motor position from map", swerveModuleSignals.get(1).get("drivePosition").getValueAsDouble());
        // SmartDashboard.putNumber("module 2 steer motor position from map", swerveModuleSignals.get(1).get("anglePosition").getValueAsDouble());
        // SmartDashboard.putNumber("module 2 cancoder position from map", swerveModuleSignals.get(1).get("angleAbsolutePosition").getValueAsDouble());
    
    }

    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds(){
        return this.getStateCopy().Speeds;  
    }
}
