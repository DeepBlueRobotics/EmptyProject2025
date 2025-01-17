package org.carlmontrobotics.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private final SparkMax rightMotor = new SparkMax(1, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private final SparkClosedLoopController pid = leftMotor.getClosedLoopController();
    public Drivetrain(){
       
    }
    public void driveMotor(double leftVoltage, double rightVoltage) {
        leftMotor.set(leftVoltage);
        rightMotor.set(-rightVoltage);
    }
}
