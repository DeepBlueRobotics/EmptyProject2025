// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.Subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  SparkMax motor1 = create.MotorControllerFactory.createSparkMax(32,config:NEO);
  Motor1.set(speed:0.25);

  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
