/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class RotateToAngle extends CommandBase implements PIDOutput {
  PIDController turnController;
  DriveTrain driveTrain;
  
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  double rotateToAngleRate;

  static final double kToleranceDegrees = 2.0;    
  static final double kTargetAngleDegrees = 90.0;

  AHRS ahrs;

  /**
   * Creates a new RotateToAngle.
   */
  /*
  public RotateToAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    try {
      ahrs = new AHRS(SPI.Port.kMXP); 
    } 
    catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    turnController.setInputRange(-180.0,  180.0);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.disable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* While this button is held down, rotate to target angle.  
    * Since a Tank drive system cannot move forward simultaneously 
    * while rotating, all joystick input is ignored until this
    * button is released.
    */
    /*
    if (!turnController.isEnabled()) {
      turnController.setSetpoint(kTargetAngleDegrees);
      rotateToAngleRate = 0; // This value will be updated in the pidWrite() method ???????
      turnController.enable();
    }
    driveTrain.setLeftMotors(rotateToAngleRate);
    driveTrain.setRightMotors(rotateToAngleRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }
}
*/