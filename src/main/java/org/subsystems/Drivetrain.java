import edu.wpi.first.wpilibj2.command.SubsystemBase;;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static org.carlmontrobotics.Constants.*;
import edu.wpi.first.math.MathUtil;

private final SparkMax rightmotor;
private final SparkMax leftmotor;
private final XboxController controller;

public newDrivertrain(XboxController controller) {
    rightmotor = MotorControllerFactory.createSparkMax(21, MotorType.kBrushless);
    leftmotor = MotorControllerFactory.createSparkMax(22, MotorType.kBrushless);
    this.controller = controller;
}

public void tankDrive(double leftSpeed, double rightSpeed) {
    leftmotor.set(leftSpeed*MOTOR_SLOWDOWN);
    rightmotor.set(rightSpeed*MOTOR_SLOWDOWN);
}

private final double MOTOR_SLOWDOWN = 0.4;

@Override
public void periodic(){
    tankDrive(controller.getLeftY(), controller.getRightY())
}

public void arcadeDrive(double speed, double rotation) {
    leftSpeed = MathUtil.clamp(speed + rotation, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(speed - rotation, -1.0, 1.0);
    leftmotor.set(leftSpeed * MOTOR_SLOWDOWN);
    rightmotor.set(-1*(rightSpeed * MOTOR_SLOWDOWN));
}

@Override
public void periodic(controller.getLeftY(), controller.getRightY()) {
}