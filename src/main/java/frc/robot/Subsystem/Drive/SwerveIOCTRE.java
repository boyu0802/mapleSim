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

import edu.wpi.first.math.geometry.Pose2d;
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
        inputs.estimatedRobotPose = this.getStateCopy().Pose;
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


    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp){
        super.addVisionMeasurement(visionRobotPose, timeStamp);
    }


    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds(){
        return this.getStateCopy().Speeds;  
    }
}
