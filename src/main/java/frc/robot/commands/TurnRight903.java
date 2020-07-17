/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurnRight903 extends PIDCommand {

  /**
   * Creates a new TurnRight903.
   */
  public TurnRight903(double targetAngle, DriveTrain driveTrain) {
    //I used https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html
    //to code this. It took me a while to understand.
    //I put each parameter on a diff line so i could explain better.
    //first param makes a new PIDController. Read up about them if you don't know what they are
    super(new PIDController(Constants.kp, Constants.ki, Constants.kd), 
    driveTrain::getGyroAngle, //this is called a method reference
    driveTrain::getMeasurement, //90 is desired turn angle. This is called the setpoint or reference of the control system.
    driveTrain::useOutput, //this is called a lambda expression. I don't get it.
    driveTrain); //requires the drive train
    
    try {
    //the instance of PIDController isn't stored in a var, so getController() is used instead of the var.
    getController().setSetpoint(90);
    //"sets the controller to be continuous (because it is an angle controller)" - from linked site
    getController().enableContinuousInput(-180, 180);
    //"sets the controller tolerance - the delta (meaning change in) tolerance ensures the robot is
    // stationary at the setpoint before it is considered as having reached the reference"
    getController().setTolerance(Constants.TurnToleranceDeg, Constants.TurnRateToleranceDegPerS);
    }
    catch(RuntimeException ex) {
      DriverStation.reportError("Error setting stuff:  " + ex.getMessage(), true);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the robot is at the setpoint, which is 90, the command ends
    return getController().atSetpoint();
  }
}
