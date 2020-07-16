/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonSRX motorLeftLShooter;
  TalonSRX motorLeftRShooter;
  TalonSRX motorRightLShooter;
  TalonSRX motorRightRShooter;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    motorLeftLShooter = new TalonSRX(Constants.motorLeftLShooterValue);
    motorLeftRShooter = new TalonSRX(Constants.motorLeftRShooterValue);
    motorRightLShooter = new TalonSRX(Constants.motorRightLShooterValue);
    motorRightRShooter = new TalonSRX(Constants.motorRightRShooterValue);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setShooterMotors(double speed){
    //left motors are negative so both wheels turn inward and shoot the ball
    motorLeftLShooter.set(ControlMode.PercentOutput, -speed);
    motorLeftRShooter.set(ControlMode.PercentOutput, -speed);
    motorRightLShooter.set(ControlMode.PercentOutput, speed);
    motorRightRShooter.set(ControlMode.PercentOutput, speed);
  }

  public void stopShooter(){
    motorLeftLShooter.set(ControlMode.PercentOutput, 0);
    motorLeftRShooter.set(ControlMode.PercentOutput, 0);
    motorRightLShooter.set(ControlMode.PercentOutput, 0);
    motorRightRShooter.set(ControlMode.PercentOutput, 0);
  }
}
