// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public static DigitalInput limitSwitch = new DigitalInput(0);
  public static SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);
  public static TimeOfFlight distanceSensor = new TimeOfFlight(5);
  public static boolean moveBack = false;
  public static boolean intakeStarted = false;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    distanceSensor.setRangingMode(RangingMode.Short, 24);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Distance", 
    distanceSensor.getRange());
    SmartDashboard.getBoolean("Move Back", moveBack);
    boolean isAnyValid = false;
    int motorSpeed = 0;
    SmartDashboard.putBoolean("Valid Distance Detected", isAnyValid);
    SmartDashboard.putBoolean("limitSwitch", limitSwitch.get());
    SmartDashboard.putNumber("motor speed", motorSpeed);
    boolean dsSeesCoral = distanceSensor.getRange() < 100;
    boolean lsSeesCoral = !limitSwitch.get();
    double lowSpeed = 0.04;
    double highSpeed = 0.2;
    double negSpeed = -0.04;

    if(dsSeesCoral){
      intakeStarted = true;
    }

    if(dsSeesCoral && !lsSeesCoral){
      if(!moveBack) motor.set(highSpeed);
      else if (moveBack) motor.set(0);
    }else if(dsSeesCoral && lsSeesCoral){
      if(!moveBack) motor.set(lowSpeed);
      else if (moveBack) motor.set(0);
    }else if(!dsSeesCoral){
      if(intakeStarted) {
        moveBack = true;
        motor.set(negSpeed);
      }
    }
}

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
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
