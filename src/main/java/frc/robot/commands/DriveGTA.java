/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

public class DriveGTA extends CommandBase {
  private final DriveTrain driveTrain;
  AHRS ahrs;
  /**
   * Creates a new GTADrive.
   */
  public DriveGTA(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.driveTrain = driveTrain;
  addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double triggerVal = Robot.m_robotContainer.getDriverRawAxis(Constants.rightTriggerValue) 
    //- Robot.m_robotContainer.getDriverRawAxis(Constants.leftTriggerValue);

    //driveTrain.setLeftMotors(triggerVal + stick);
    //driveTrain.setRightMotors(triggerVal - stick); 

    //I think this just multiplies the two values. Fix this later.
    double stick = Utilities.scale(Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX), 
    Constants.turningRate);

    //squaring the trigger values make them less sensitive when you barely press down on them. 
    double rightTriggerSquared = Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger)
      *Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger);
    double leftTriggerSquared = Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger)
      *Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger);
    
    double triggerVal = rightTriggerSquared - leftTriggerSquared;

    driveTrain.setLeftMotors(triggerVal + stick);
    driveTrain.setRightMotors(triggerVal - stick);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
