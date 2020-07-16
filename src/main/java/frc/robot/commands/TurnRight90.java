/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnRight90 extends CommandBase {
  AHRS ahrs;
  DriveTrain driveTrain;
  private boolean finished = false;

  //these doubles might need to go somewhere else
  double targetAngle = 90;
  double kp = 0.01;
  //double ki = 0.01;
  //double kd = 0.01;
  double leftCommand = 0;
  double rightCommand = 0;
  double angleLeeway = 20;
  //PIDController controller = new PIDController(kp, ki, kd);

  /**
   * Creates a new Turn90.
   */
  public TurnRight90(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    }
    catch(RuntimeException exception) {
      DriverStation.reportError("Error instantiating navX MXP:  " + exception.getMessage(), true);
    }
    //ahrs.reset();p    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (ahrs.getAngle() < targetAngle)
    {
      driveTrain.setLeftMotors(1);
      driveTrain.setRightMotors(0);
    }
    else if (ahrs.getAngle() > targetAngle)
    {
      driveTrain.setLeftMotors(1);
      driveTrain.setRightMotors(0);
    }
    else
    {
      driveTrain.setLeftMotors(0);
      driveTrain.setRightMotors(0);
      finished = true;
    }
    */

    //As of 7/14/20, this command turns left for like half a sec, then turns right like one sec the first
    //time that it's run. The next times, it barely moves any to the right.
    double steeringAdjust = ahrs.getAngle() * kp;

    leftCommand -= steeringAdjust;
    rightCommand += steeringAdjust;

    driveTrain.setLeftMotors(leftCommand / 30);
    driveTrain.setRightMotors(rightCommand / 30);

    if (ahrs.getAngle() >= (targetAngle - angleLeeway) && ahrs.getAngle() <= (targetAngle + angleLeeway))
    {
      finished = true;  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
