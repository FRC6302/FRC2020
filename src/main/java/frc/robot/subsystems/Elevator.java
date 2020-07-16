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

public class Elevator extends SubsystemBase {
  TalonSRX motorElevator;
  
  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    motorElevator = new TalonSRX(Constants.motorElevatorValue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorMotor(double speed){
    motorElevator.set(ControlMode.PercentOutput, speed);
  }

  public void stopElevator(){
    motorElevator.set(ControlMode.PercentOutput, 0);
  }
}
