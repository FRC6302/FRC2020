/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveTrain extends SubsystemBase {
  //the MechanumDrive constructor doesn't take in TalonSRX arguements so I had to use Talon. Idk if it'll work
  public Talon motorL1;
  public Talon motorL2;
  public Talon motorR1;
  public Talon motorR2;

  public MecanumDriveTrain() {
    motorL1 = new Talon(Constants.motorL1Value);
    motorL2 = new Talon(Constants.motorL2Value);
    motorR1 = new Talon(Constants.motorR1Value);
    motorR2 = new Talon(Constants.motorR2Value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void stopDrive(){
    motorL1.set(0);
    motorL2.set(0);
    motorR1.set(0);
    motorR2.set(0);
  }
}
