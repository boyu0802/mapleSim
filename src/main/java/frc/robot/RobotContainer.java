// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SimDrivetrainConstant;
import frc.robot.Subsystem.Drive.SwerveIOCTRE;
import frc.robot.Subsystem.Drive.SwerveIOSim;
import frc.robot.Subsystem.Drive.SwerveSubsystem;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private SwerveSubsystem swerveSubsystem;
  
      private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband( DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND*0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            

  public RobotContainer() {
    if(Robot.isReal()){
      swerveSubsystem = new SwerveSubsystem(new SwerveIOCTRE(DrivetrainConstants.SWERVE_DRIVETRAIN_CONSTANTS, DrivetrainConstants.MODULE_CONSTANTS));
    } else {
      swerveSubsystem = new SwerveSubsystem(new SwerveIOSim(SimDrivetrainConstant.DrivetrainConstants, SimDrivetrainConstant.MODULE_CONSTANTS));
      
    }
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.applyRequest(
        () -> drive.withVelocityX(-controller.getLeftY()*DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND)
                  .withVelocityY(-controller.getLeftX()*DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND)
                  .withRotationalRate(-controller.getRightX()*DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND))
    );
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
