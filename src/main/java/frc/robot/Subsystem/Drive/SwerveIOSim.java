package frc.robot.Subsystem.Drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SimDrivetrainConstant;
import frc.robot.Util.MapleSimSwerveDrivetrain;

public class SwerveIOSim extends SwerveIOCTRE{
    private final double SIM_LOOPTIME = 0.02;
    private MapleSimSwerveDrivetrain mapleSim = null;
    private Notifier notifier = null;
    private SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[4];

    public SwerveIOSim(SwerveDrivetrainConstants drivetrainConstants, @SuppressWarnings("rawtypes") SwerveModuleConstants... moduleConstants) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(moduleConstants));
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread(){
        mapleSim = new MapleSimSwerveDrivetrain(
            edu.wpi.first.units.Units.Second.of(SIM_LOOPTIME),
            DrivetrainConstants.DRIVETRAIN_SIMULATED_CONFIGS,
            getPigeon2(), 
            getModules(),
            SimDrivetrainConstant.FrontLeft,
            SimDrivetrainConstant.FrontRight,
            SimDrivetrainConstant.BackLeft,
            SimDrivetrainConstant.BackRight
            );
        
        notifier = new Notifier(mapleSim::update);
        notifier.startPeriodic(SIM_LOOPTIME);
    }

    public void updateSwerveInputs(SwerveIOInputs inputs){
        inputs.gyroAngle = mapleSim.mapleSimDrive.getGyroSimulation().getGyroReading();
        inputs.moduleStates = getState().ModuleStates;
        inputs.modulePositions = this.getStateCopy().ModulePositions;
        inputs.timeStamp = this.getStateCopy().Timestamp;
        inputs.absoluteRobotPose = mapleSim.mapleSimDrive.getSimulatedDriveTrainPose();
        inputs.estimatedRobotPose = this.getStateCopy().Pose;
        inputs.targetStates = this.getStateCopy().ModuleTargets;
        inputs.fieldRelativeChassisSpeeds = mapleSim.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    public SwerveDriveSimulation getSimulatedDriveTrain(){
        return mapleSim.mapleSimDrive;
    }

}
