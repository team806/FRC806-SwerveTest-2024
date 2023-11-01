// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.drivetrain;
import frc.robot.swerve.swervedrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  drivetrain m_drivetrain = new swervedrive(
    new Translation2d[] {new Translation2d(),new Translation2d(),new Translation2d(),new Translation2d()}, 
    new CANSparkMax[] {new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless)}, 
    new CANSparkMax[] {new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless)}, 
    new CANcoder[] {new CANcoder(0),new CANcoder(0),new CANcoder(0),new CANcoder(0)}, 
    new ADIS16470_IMU(),
    new Pose2d(null, null)
  );
  
  @Override
  public void robotInit() {
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
}
