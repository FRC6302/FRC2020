/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MecanumDriveTrain;

public class DriveMecanum extends CommandBase {
  MecanumDriveTrain mecanumDriveTrain;
  double ySpeed;
  double xSpeed;
  double zRotation;
  
  MecanumDrive mecanumDrive;

  public DriveMecanum(MecanumDriveTrain mecanumDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.mecanumDriveTrain = mecanumDriveTrain;
  addRequirements(mecanumDriveTrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //cam mode that is normal and not dark
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ySpeed = Robot.robotContainer.getDriverRawAxis(Constants.leftStickY);
    xSpeed = Robot.robotContainer.getDriverRawAxis(Constants.leftStickX);
    zRotation = Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger) 
    - Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger);

    mecanumDrive = new MecanumDrive(mecanumDriveTrain.motorL1, 
    mecanumDriveTrain.motorL2, mecanumDriveTrain.motorR1, mecanumDriveTrain.motorR2);
    //might have to switch the positions of xSpeed and ySpeed
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mecanumDriveTrain.stopDrive();   
    //idk why i have to close it but i do
    mecanumDrive.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
