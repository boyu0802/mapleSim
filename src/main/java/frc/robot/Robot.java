// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);    

    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    }else{
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (Robot.isSimulation()) {

      SimulatedArena.getInstance().placeGamePiecesOnField();
       // Add a coral piece to make life easier in sim.
       SimulatedArena.getInstance()
          .addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 2, Rotation2d.kZero)));
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


  @Override
  public void simulationPeriodic() {
      Logger.recordOutput(
              "FieldSimulation/Coral",
              SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
      Logger.recordOutput(
              "FieldSimulation/Algae",
              SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
