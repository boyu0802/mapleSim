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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SimDrivetrainConstant;
import frc.robot.Subsystem.AlgaeIntake.IntakeIOSIM;
import frc.robot.Subsystem.AlgaeIntake.IntakeSubsystem;
import frc.robot.Subsystem.CoralAlgaeIntake.CoralIOSIM;
import frc.robot.Subsystem.CoralAlgaeIntake.CoralSubsytem;
import frc.robot.Subsystem.Drive.SwerveIOCTRE;
import frc.robot.Subsystem.Drive.SwerveIOSim;
import frc.robot.Subsystem.Drive.SwerveSubsystem;
import frc.robot.Subsystem.Elevator.ElevatorIOSIM;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;
import frc.robot.Subsystem.Hanger.HangerIOSIM;
import frc.robot.Subsystem.Hanger.HangerSubsystem;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private SwerveSubsystem swerveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private HangerSubsystem hangerSubsystem;
  private CoralSubsytem coralSubsytem;
  private RobotState robotState = RobotState.getInstance();

      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband( DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND*0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            

  public RobotContainer() {
    

    if(Robot.isReal()){
      swerveSubsystem = new SwerveSubsystem(new SwerveIOCTRE(DrivetrainConstants.SWERVE_DRIVETRAIN_CONSTANTS, DrivetrainConstants.MODULE_CONSTANTS));
    } else {
      swerveSubsystem = new SwerveSubsystem(new SwerveIOSim(SimDrivetrainConstant.DrivetrainConstants, SimDrivetrainConstant.MODULE_CONSTANTS));
      intakeSubsystem = new IntakeSubsystem(new IntakeIOSIM(Constants.AlgaeIntakeConstants.PIVOT_MOTOR_CONFIG, Constants.AlgaeIntakeConstants.ROLLER_MOTOR_CONFIG));
      elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSIM(Constants.ElevatorConstants.ELEVATOR_SIM_CONFIG_L, Constants.ElevatorConstants.ELEVATOR_SIM_CONFIG_R));
      hangerSubsystem = new HangerSubsystem(new HangerIOSIM(Constants.HangerConstants.HANG_SIM_CONFIG, Constants.HangerConstants.HANG_ENCODER_CONFIG));
      coralSubsytem = new CoralSubsytem(new CoralIOSIM(Constants.coralWristConstants.CORAL_WRISTCONFIG));
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


    controller.a().onTrue(intakeSubsystem.toIntakePosition());
    controller.b().onTrue(intakeSubsystem.toDefaultPosition());
    controller.x().onTrue(elevatorSubsystem.toL1());
    controller.y().whileTrue(elevatorSubsystem.toZero());
    controller.povUp().onTrue(hangerSubsystem.toCage());
    controller.povDown().onTrue(hangerSubsystem.hang());
    controller.povRight().onTrue(coralSubsytem.toIntake());
    controller.povLeft().onTrue(coralSubsytem.toDefault());

  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
