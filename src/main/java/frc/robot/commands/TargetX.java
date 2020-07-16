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


public class TargetX extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;
  
  /**
   * Creates a new LimelightStuff.
   */
  public TargetX(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    //idk how these are different than the lines below but they might be needed

    //camera controls-might go somewhere else besides execute
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    //changing camMode can be used to switch between the normal cam and the darkened targeting mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    //NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    //TRY CHANGING THE ZEROES AND SEE IF ANYTHING HAPPENS!!!!
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);

    //float Kp = -0.1f;  // Proportional control constant
    //boolean buttonVal = Robot.m_robotContainer.getButton(Constants.limelightDriveButtonValue);
    // while (x > Constants.limelightTXDeadzone)
    // {
    //   driveTrain.setLeftMotors(Constants.limelightTargetXYSpeed*-1);
    //   driveTrain.setRightMotors(Constants.limelightTargetXYSpeed);
    //   //driveTrain.setRightMotors(x*Constants.limelightTargetLineUpXYScaling);
        
    // }
    // while (x < Constants.limelightTXDeadzone) 
    // {
    //   driveTrain.setLeftMotors(Constants.limelightTargetXYSpeed);
    //   //driveTrain.setLeftMotors(x*Constants.limelightTargetLineUpXYScaling);
    //   driveTrain.setRightMotors(Constants.limelightTargetXYSpeed*-1);
    // }
   

    double leftChange = 0;
    leftChange = leftChange + x;
    double rightChange = 0;
    rightChange = rightChange - x;
    driveTrain.setLeftMotors(leftChange);
    driveTrain.setRightMotors(rightChange);
    if (x < Constants.limelightTXDeadzone && x > Constants.limelightTXDeadzone*-1) 
    {
      driveTrain.stopDrive();
      //NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1);
      //takes 2 snapshots a second that can tell you if the targeting is right. Supposed to
      //automatically post the snapshots on the limelight 5801 thing i think
      finished = true;
    }

    // if (x == 0) 
    // {
    //   driveTrain.stopMotors();
    //   finished = true;
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
