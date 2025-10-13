package frc.robot.Subsystem.Drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util.SubsystemDataProcessor.IoRefresher;

public interface SwerveIO extends IoRefresher {

    @AutoLog
    class SwerveIOInputs {    
        public Pose2d absoluteRobotPose = new Pose2d();
        public Pose2d estimatedRobotPose = new Pose2d();
        public SwerveModuleState[] moduleStates;
        public ChassisSpeeds fieldRelativeChassisSpeeds;
        public SwerveModuleState[] targetStates;
        public SwerveModulePosition[] modulePositions;
        public Rotation2d gyroAngle =  new Rotation2d();
        public double timeStamp;
    }

    @AutoLog
    class ModuleIOInputs{
        public double driveId;
        public double drivePosition;
        public double driveVelocity;
        public double driveAppliedVolts;
        public double steerId;
        public Rotation2d steerAbosolutePosition;
        public double steerVelocity;
        public double steerAppliedVolts;
    }

    public default void updateSwerveInputs(SwerveIOInputs inputs) {}

    public default void updateModuleInputs(ModuleIOInputs... inputs){}

    public default Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {return null;}

    public default void periodic() {}

    @Override
    public default void refreshData() {}
}
