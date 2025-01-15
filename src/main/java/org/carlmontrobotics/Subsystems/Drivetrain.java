package org.carlmontrobotics.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.carlmontrobotics.lib199.MotorConfig;
public class Drivetrain extends SubsystemBase {
    private final SparkMax sexm = new SparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder enc = sexm.getEncoder();
    private final SparkClosedLoopController pid = sexm.getClosedLoopController();
    public Drivetrain(){
       
    }
    public void driveMotor(double voltage) {
        sexm.set(voltage);
    }
}
