/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase {
  /*
  NOTE TO NEW CODERS: YOU SHOULD PROBABLY DO THE APCSA CODEHS COURSE BEFORE TRYING TO CODE 
  FOR ROBOTICS. IT WAS THE MOST HELPFUL COURSE I'VE DONE, AND THE BEGINNING IS FUN WITH KAREL.
  MAKE SURE YOU COMPLETELY UNDERSTAND ALL THE STUFF ABOUT CLASSES, OBJECTS, AND METHODS. YOU CAN
  PROBABLY SKIP SOME OF THE STRING STUFF, BUT IT'S STILL GOOD TO KNOW. GOOD LUCK :) -SAMUEL
  */

  TalonSRX motorL1;
  TalonSRX motorL2;
  TalonSRX motorR1;
  TalonSRX motorR2;
  //AHRS is the NavX gyro class
  AHRS ahrs;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {    
    motorL1 = new TalonSRX(Constants.motorL1Value);
    motorL2 = new TalonSRX(Constants.motorL2Value);
    motorR1 = new TalonSRX(Constants.motorR1Value);
    motorR2 = new TalonSRX(Constants.motorR2Value);
    //motorL1.setInverted(false);
    //motorL2.setInverted(false);
    //motorR1.setInverted(false);
    //motorR2.setInverted(false);

    //try catch is used here to let the driver know if the navX was set up wrong
    try {
      //Kauai labs site said SPI is the best port if the NavX is mounted to RoboRio, which it is.
      ahrs = new AHRS(SPI.Port.kMXP);
    }
    catch(RuntimeException exception) {
      DriverStation.reportError("Error instantiating navX MXP:  " + exception.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setLeftMotors(double speed){
    motorL1.set(ControlMode.PercentOutput, speed);
    motorL2.set(ControlMode.PercentOutput, speed);
  }

  //right motors have inverted speed bc of how the motors are oriented on robot
  public void setRightMotors(double speed){
    motorR1.set(ControlMode.PercentOutput, -speed);
    motorR2.set(ControlMode.PercentOutput, -speed);
  }

  public double getGyroAngle(){
    return ahrs.getAngle();
  }

  public void usePIDOutput(double output){
    //right is inverted so that the robot turns right
    setLeftMotors(output / 30);
    setRightMotors(-output / 30);
  }

  public boolean isWithinThreshold(double targetAngle){
    double error = targetAngle - ahrs.getAngle();
    double threshold = Constants.PIDErrorThreshold;
    if (error > threshold)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  public void rotateRight(double targetAngle){
    double kP = 0.025;
    double error = targetAngle - ahrs.getAngle();
    double threshold = Constants.PIDErrorThreshold;
    double leftCommand = 0;
    double rightCommand = 0;

    while (error > threshold)
    {
      error = targetAngle - ahrs.getAngle();
      double steeringAdjust = error * kP;
      leftCommand += steeringAdjust;
      rightCommand -= steeringAdjust;
      
      setLeftMotors(leftCommand / 30);
      setRightMotors(rightCommand / 30);
    }
    //stopDrive();
  }

  public void rotateLeft(double targetAngle){
    double kP = 0.025;
    double error = targetAngle - ahrs.getAngle();
    double threshold = Constants.PIDErrorThreshold;
    double leftCommand = 0;
    double rightCommand = 0;

    while (error > threshold)
    {
      error = targetAngle - ahrs.getAngle();
      double steeringAdjust = error * kP;
      leftCommand -= steeringAdjust;
      rightCommand += steeringAdjust;
      
      setLeftMotors(leftCommand / 30);
      setRightMotors(rightCommand / 30);
    }
    //stopDrive();
  }

  public void stopDrive(){
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }
  
}
