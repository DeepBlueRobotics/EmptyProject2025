package org.carlmontrobotics.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(4, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private final SparkMax rightMotor = new SparkMax(2, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private final SparkClosedLoopController pid = leftMotor.getClosedLoopController();
    public Drivetrain(){
        leftMotor.set(1);
    }
    public void driveMotor(double leftVoltage, double rightVoltage) {
        SmartDashboard.putNumber("Wei on meter: ", leftVoltage);
        leftMotor.set(leftVoltage);
        rightMotor.set(rightVoltage);
    }
}
