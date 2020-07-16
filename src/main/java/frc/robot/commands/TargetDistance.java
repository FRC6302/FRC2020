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

public class TargetDistance extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;
  /**
   * Creates a new TargetDistance.
   */
  public TargetDistance(DriveTrain dt) {
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
    //idk how these are different than the getEntry below but it might be needed

    //camera controls
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    //changing camMode can be used to switch between the normal cam and the special limelight darkened mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    //TRY CHANGING THE ZEROES AND SEE IF ANYTHING HAPPENS!!!!
    //double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    //while (area < Constants.limelightSpecificArea)
    //{
      //driveTrain.setLeftMotors(Constants.limelightTargetDistanceSpeed);
      //driveTrain.setRightMotors(Constants.limelightTargetDistanceSpeed);
    //}
    //while (area > Constants.limelightSpecificArea)
    // {
    //   driveTrain.setLeftMotors(Constants.limelightTargetDistanceSpeed*-1);
    //   driveTrain.setRightMotors(Constants.limelightTargetDistanceSpeed*-1);
    // }
   

    double distanceChange = 0;
    distanceChange = distanceChange + area;
    driveTrain.setLeftMotors(distanceChange);
    driveTrain.setRightMotors(distanceChange);
    if (area < Constants.limelightSpecificArea && area > Constants.limelightSpecificArea*-1){
      driveTrain.setLeftMotors(0);
      driveTrain.setRightMotors(0);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
