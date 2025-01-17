// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.Commands;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new drive. */
  private final Drivetrain drivetrain;
  private final double joyLeftY;
  private final double joyRightY;
  public Drive(Drivetrain drivetrain, DoubleSupplier joyLeftY, DoubleSupplier joyRightY) {
    this.drivetrain = drivetrain;
    this.joyLeftY = joyLeftY.getAsDouble();
    this.joyRightY = joyRightY.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveMotor(joyLeftY, joyRightY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
