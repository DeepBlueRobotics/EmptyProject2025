import edu.wpilib.first.wpilib2.command.SubsystemBase;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Motor extends SubsystemBase() [
    private final int MOTORID = 3; //Check this before running!
    private CanSparkMax motor = MotorControllerFactory.createSparkMax(MOTORID, MotorConfig.NEO);

    public Motor() {
        //Just have this because WPILib errors out when you don't!
    }

    @Override
    public void periodic() {
        motor.set(0.1);
    }
]