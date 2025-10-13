package frc.robot.Subsystem.Drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Util.SubsystemDataProcessor;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DrivetrainConstants.MODULE_POSITIONS);
    private final double DEADBAND = 0.03;

    SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    ModuleIOInputsAutoLogged FLinputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged BLinputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged BRinputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged FRinputs = new ModuleIOInputsAutoLogged();

    private SwerveIO io = new SwerveIO(){};
    private Object lock = new Object();


    public SwerveSubsystem(SwerveIO io){
        this.io = io;
        SubsystemDataProcessor.createSubsystemDataProcessor(io,
            () ->{
                synchronized(lock){
                    io.updateModuleInputs(FLinputs,BLinputs,BRinputs,FRinputs);
                }
        });
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this);
    }


    @Override
    public void periodic(){
        io.updateSwerveInputs(inputs);
        synchronized(lock){
            Logger.processInputs("Subsystem/SwerveDrivetrain", inputs);
            Logger.processInputs("Subsystem/Swerve/Modules/FL", FLinputs);
            Logger.processInputs("Subsystem/Swerve/Modules/BL", BLinputs);
            Logger.processInputs("Subsystem/Swerve/Modules/BR", BRinputs);
            Logger.processInputs("Subsystem/Swerve/Modules/FR", FRinputs);
        }
        // io.periodic();


    }

 
}
