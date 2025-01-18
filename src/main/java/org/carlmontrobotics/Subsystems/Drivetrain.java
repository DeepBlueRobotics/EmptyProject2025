// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.Subsystems;
import org.carlmontrobotics.lib199.swerve.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final SparkMax[] driveMotors = new SparkMax[] {null, null, null, null};
  private final SparkMax[] turnMotors = new SparkMax[] {null, null, null, null};

  private final moduleFL;
  private final moduleFR;
  private final moduleBL;
  private final moduleBR;
  public Drivetrain() {
    moduleFL = new SwerveModule(null, SwerveModule.ModuleType.FL, driveMotors[0] = new SparkMax(0, MotorType.kBrushless), turnMotors[0] = new SparkMax(1, MotorType.kBrushless), null, 0, null, null)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
