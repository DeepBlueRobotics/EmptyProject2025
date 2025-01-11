package org.carlmontrobotics.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    public Motor() {

    }
    CANSparkMax motor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO/NEO_550);
    


    @Override
    public void periodic() {
        motor.set(0.5);
    }

    
}