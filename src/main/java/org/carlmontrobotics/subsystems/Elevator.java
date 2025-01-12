package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.carlmontrobotics.Constants.OI.ElevatorC;

public class Elevator extends SubsystemBase{

    CANSparkMax motorMaster = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO);
    CANSparkMax motorFollower = MotorControllerFactory.createSparkMax(1, MotorConfig.NEO);
    SparkAbsoluteEncoder absoluteEncoder = motorMaster.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private PIDController pidController;
    private TrapezoidProfile elevatorProfile;
    private TrapezoidProfile.State setpoint = getCurrentState();
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    public Elevator(){
        pidController = new PIDController(ElevatorC.kP, ElevatorC.kI, ElevatorC.kD);
        pidController.setTolerance(ElevatorC.pidTolerance);
        elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorC.MAX_VEL, ElevatorC.MAX_ACCEL));

        setpoint = getCurrentState();
        goalState = getCurrentState();
        setElevatorTarget(goalState.position);
        
    }

    @Override
    public void periodic(){
        driveElevator();
    }


    public TrapezoidProfile.State getCurrentState(){
        return new TrapezoidProfile.State(getElevatorHeight(), getElevatorVel());
    }

    public void driveElevator(){
        setpoint = elevatorProfile.calculate(ElevatorC.kD, setpoint, goalState);

        double currentPos = absoluteEncoder.getPosition();
        double motorOutput = pidController.calculate(currentPos);
        motorMaster.set(motorOutput);
        motorFollower.follow(motorMaster);
    }


    public double getElevatorHeight(){
        return absoluteEncoder.getPosition();

        // TODO: DO THE MATH
    }

    public double getElevatorVel(){
        return absoluteEncoder.getVelocity();
    }

    public void setElevatorTarget(double targetHeight){
        goalState.position = targetHeight;
        goalState.velocity = 0;
    }


}
