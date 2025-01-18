// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdhitArm extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(3, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
