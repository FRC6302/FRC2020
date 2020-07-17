/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //motors
	public static final int motorL1Value = 0;
	public static final int motorL2Value = 1;
	public static final int motorR1Value = 2;
    public static final int motorR2Value = 3;
    public static final int motorLeftLShooterValue = 4;
	public static final int motorLeftRShooterValue = 5;
	public static final int motorRightLShooterValue = 6;
    public static final int motorRightRShooterValue = 7;
    public static final int motorIntakeValue = 8;
    public static final int motorFeederValue = 9;
    public static final int motorElevatorValue = 90;
    public static final int motorSpinnerValue = 91;

    //controller values
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int leftStickX = 90;
    public static final int leftStickY = 91;
    public static final int rightStickX = 92;
    public static final int rightStickY = 5;
    public static final int aButton = 1;
    public static final int bButton = 2;
    public static final int xButton = 3;
    public static final int yButton = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;

    //controller ports
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    
    //motor speeds
    public static final double shooterSpeed = 1.0;
    public static final double intakeSpeed = 0.3;	

    //Move command
    public static final int MoveTime = 1;
    public static final double leftMotorsMoveSpeed = 0.3;
    public static final double rightMotorsMoveSpeed = 0.3;

    //TimedShoot command
    public static final double timedShootTime = 2;
    public static final double timedShootSpeed = 1.0;
    
    //Feed command
    public static final double FeedTime = 3;
    public static final double feederMotorMoveSpeed = 0.5;
    
    //TimedLift command
    public static final double timedLiftTime = 0.2;
    public static final double timedLiftSpeed = 0.2;
    
    //Spin command
    public static final double spinSpeed = 1.0;
    public static final double spinTime = 3.7;

    //limelight
    public static final double limelightTXDeadzone = 0.5;
    public static final double limelightTargetXScaling = 1;
    public static final double limelightSpecificArea = 27;
    public static final double limelightTargetDistanceSpeed = 0.3;
    public static final double limelightSeekSpeed = 0.2;
    public static final double limelightTargetXSpeed = 0.2;

    //gyro
    public static final double PIDErrorThreshold = 2;
    public static final double kp = 0.011;
    public static final double ki = 0.01;
    public static final double kd = 0.01;
	public static final double TurnToleranceDeg = 5;
    public static final double TurnRateToleranceDegPerS = 5; //idk what this should be
    
	
    //camera resolutions
	public static final int camera1XAxisResolution = 320;
    public static final int camera1YAxisResolution = 240;
    //these are the values that the new vid(part 7) used. I think we use a lot lower resolution for max fps
    
    //miscellaneous
    public static final double turningRate = 0.5;
    public static final double deadzone = 0.4;

    //driver contoller buttons
    public static final int limelightTargetButton = Constants.aButton;
    public static final int moveButton = Constants.xButton;
    public static final int turnRight90Button = Constants.aButton;
	public static final int turnRight902Button = Constants.bButton;
    public static final int turnRight903Button = Constants.yButton;
    
    //operator controller buttons
	//public static final int shootButton = Constants.xButton;
    //public static final int feedButton = Constants.aButton;
	//public static final int shootAndFeedButton = Constants.bButton;
	public static final int intakeButton = Constants.rightBumper;
	//public static final int timedLiftButton = Constants.yButton;
	public static final int spinButton = Constants.leftBumper;
    public static final int liftUpAndDownAxis = Constants.rightStickY;
	
}
