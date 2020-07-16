/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SeekLeft extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;

  /**
   * Creates a new SeekLeft.
   */
  public SeekLeft(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //camera controls
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    //changing camMode can be used to switch between the normal cam and the darkened targeting mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    //idk how these are different than the lines below but they might be needed

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    //NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
    
    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightV", v);

    double steeringAdjust;
    if (v == 0) //target not found has a value of 0
    {
      //driveTrain.setLeftMotors(Constants.limelightSeekSpeed);
      //driveTrain.setRightMotors(Constants.limelightSeekSpeed*-1);
      steeringAdjust = Constants.limelightSeekSpeed;
    }
    else 
    {
      //driveTrain.stopDrive();
      //finished = true;
      steeringAdjust = x;
    }

    double leftCommand = 0;
    leftCommand += steeringAdjust;
    double rightCommand = 0;
    rightCommand -= steeringAdjust;
    driveTrain.setLeftMotors(leftCommand/3);
    driveTrain.setRightMotors(rightCommand/3);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    //changes back to normal cam mode
    //driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
