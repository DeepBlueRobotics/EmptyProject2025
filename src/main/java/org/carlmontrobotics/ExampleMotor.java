// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleMotor extends SubsystemBase {
  /** Creates a new ExampleMotor. */
  //Create a SparkMax Controller for NEO or NEO550
  SparkMax exampleMotor1 = new MotorControllerFactory.createMotor();

  public ExampleMotor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
