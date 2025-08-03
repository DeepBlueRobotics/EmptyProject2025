// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleMotor extends SubsystemBase {
  /** Creates a new ExampleMotor. */
  //Create a SparkMax Controller for NEO or NEO550
  int id1 = 1;
  SparkMax exampleMotor1 = MotorControllerFactory.createSparkMax(id1, MotorConfig.NEO); //if it would be NEO550 you would put NEO_550 as the config
  //Create a SparkFlex
  int id2 = 2;
  SparkFlex exampleMotor2 = MotorControllerFactory.createSparkFlex(id2);

  SparkMaxConfig exampleConfig1 = new SparkMaxConfig();
  SparkFlexConfig exampleConfig2 = new SparkFlexConfig();

  public ExampleMotor() {
    configureMotors();
  }

  /**
   * Configures motors
   */
  private void configureMotors() {
    exampleConfig1
    .idleMode(IdleMode.kBrake)//or can do kCoast
    .inverted(false);// or can do true
    exampleConfig1.encoder
    .positionConversionFactor(360)//Generally the relative motor counts in rotations so if you want degrees you can do this
    .velocityConversionFactor(1);
    exampleConfig1.closedLoop.pid(5,0,0);
    exampleMotor1.configure(exampleConfig1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the motor with voltage percentage 
   * @param percentage a double ranging from -1.0 to 1.0 indicating the percent of power you want the motor to use
   */
  public void runMotorPercentage(double percentage) {
    exampleMotor1.set(percentage);
    exampleMotor2.set(percentage);
    //Method is the same for SparkMax and SparkFlex
  }
  /**
   * Runs motor with a defined amount of volts, 
   * the configured SmartCurrent limiter would limit this if the requested amount of voltage would kill the motor
   * @param voltage generally between -12 and 12
   */
  public void runMotorVoltage(double voltage) {
    exampleMotor1.setVoltage(voltage);
    exampleMotor2.setVoltage(voltage);
    //as you can see this is also similar for both object types
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
