// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Motor extends SubsystemBase {

  private int port = 1;
  
  public SparkFlex spark;
  public SparkClosedLoopController pidcon;
  public SparkFlexConfig currConfig = new SparkFlexConfig();
  private double[] pid = {0,0,0};
  private boolean pidenabled = false;
  private boolean ppide = false;//periodic pid?

  public double goal = 0;

  public void config(){
    this.spark.configure(currConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Motor() {
    currConfig.encoder
      .positionConversionFactor(360);
    currConfig.closedLoop
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0,360)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    this.spark = new SparkFlex(port, MotorType.kBrushless);
    config();
    // setport(port);

    //config works :)
    // spark.set(.2);


    //initla data
    SmartDashboard.putNumber("spark port", port);

    SmartDashboard.putBoolean("pid enabled?", pidenabled);
    SmartDashboard.putBoolean("periodic PID?", ppide);

    SmartDashboard.putNumber("p", pid[0]);
    SmartDashboard.putNumber("i", pid[1]);
    SmartDashboard.putNumber("d", pid[2]);

    SmartDashboard.putNumber("setpoint", goal);


    //commands
    SmartDashboard.putData("update spark port",new InstantCommand(()->{
      setport((int)SmartDashboard.getNumber("spark port", port));
    }));
    SmartDashboard.putData("update PID constants",new InstantCommand(()->{
      setpid(
        SmartDashboard.getNumber("p", pid[0]),
        SmartDashboard.getNumber("i", pid[1]),
        SmartDashboard.getNumber("d", pid[2])
      );
    }));
    SmartDashboard.putData("goto setpoint",new InstantCommand(()->{
      gotogoal();
    }));
  }

  public Motor setport(int sparkmaxport){
    this.spark = new SparkFlex(sparkmaxport, MotorType.kBrushless);
    this.pidcon = this.spark.getClosedLoopController(); 
    config(); return this;
  }
  public Motor setpid(double p, double i, double d){
    this.currConfig.closedLoop.pid(p, i, d);
    config(); return this;
  }

  //goal pos, not encoder pos
  // public void setpos(double pos){//DOES NOT WORK
  //   enablepid();
  //   this.pidcon.setReference(pos, ControlType.kPosition);
  // }
  // public void set(double percentagePower){
  //   this.spark.set(percentagePower);
  //   disablepid();
  // }
  public double getpos(){
    return this.spark.getEncoder().getPosition();
  }

  //for internal use to manually disable PID
  // public void disablepid(){
  //   pid[0] = this.spark.configAccessor.closedLoop.getP();
  //   pid[1] = this.spark.configAccessor.closedLoop.getI();
  //   pid[2] = this.spark.configAccessor.closedLoop.getD();
  //   setpid(0,0,0);
  //   this.pidenabled=false;
  // }
  // public void enablepid(){
  //   if (!pidenabled)
  //     setpid(pid[0],pid[1],pid[2]);
  //   this.pidenabled=true;
  // }

  //for either mode
  public void gotogoal(){
    this.spark.set(goal);
    // if (pidenabled){
    //   // setpos(goal);
    // } else {
    //   // set(goal);
    //   this.spark.set(goal);
    // }
  }

  @Override
  public void periodic() {
    //this.spark.set(goal);
    //  System.out.println("hi");
    //^ works

    // if (ppide){
    //   setpos(goal);
    // }

    // gotogoal();

    pidenabled = SmartDashboard.getBoolean("pid enabled?", pidenabled);
    ppide = SmartDashboard.getBoolean("periodic PID?", ppide);
    goal = SmartDashboard.getNumber("setpoint", .4);

    SmartDashboard.putNumber("encoderPOS",getpos());

    // SmartDashboard.putNumber("encoder", this.spark.getEncoder().getPosition());
    // SmartDashboard.putNumber("absolute", this.spark.getAbsoluteEncoder().getPosition());
    // absolute doesnt change :(

    SmartDashboard.putNumber("encoderVEL",this.spark.getEncoder().getVelocity());
  }
}
