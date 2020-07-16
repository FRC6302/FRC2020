/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.TimedShoot;
import frc.robot.commands.TurnRight90;
import frc.robot.commands.TurnRight902;
import frc.robot.commands.TurnRight903;
import frc.robot.commands.AutonLeft;
import frc.robot.commands.AutonMiddle;
import frc.robot.commands.AutonRight;
import frc.robot.commands.Feed;
import frc.robot.commands.DriveGTA;
import frc.robot.commands.LiftUpAndDown;
import frc.robot.commands.TargetX;
import frc.robot.commands.TargetingSequence;
import frc.robot.commands.Move;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAndFeed;
import frc.robot.commands.Spin;
import frc.robot.commands.ParallelShootAndFeed;
import frc.robot.commands.SeekLeft;
import frc.robot.commands.SeekRight;
import frc.robot.commands.TakeInBalls;
import frc.robot.commands.TargetDistance;
import frc.robot.commands.TimedLift;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 * 
 */

public class RobotContainer {
//public class RobotContainer<Time> {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  public static XboxController driverController;
  public static XboxController operatorController;

  private final DriveTrain driveTrain;
  private final DriveGTA driveGTA;
  //private final MecanumDriveTrain mecanumDriveTrain;
  //private final DriveMecanum driveMecanum;

  private final Elevator elevator;
  private final LiftUpAndDown liftUpAndDown;
  private final TimedLift timedLift;

  private final Shooter shooter;
  private final Shoot shoot;
  private final TimedShoot timedShoot;

  private final Feeder feeder;
  private final Feed feed;
  
  private final ParallelShootAndFeed parallelShootAndFeed;

  private final ShootAndFeed shootAndFeed;
  
  private final Move move;

  private final Limelight limelight;
  private final TargetX targetX;
  private final TargetDistance targetDistance;
  private final SeekLeft seekLeft;
  private final SeekRight seekRight;
  private final TargetingSequence targetingSequence;

  private final TurnRight90 turnRight90;
  private final TurnRight902 turnRight902;
  private final TurnRight903 turnRight903;

  private final Intake intake;
  private final TakeInBalls takeInBalls;
  
  private final Spinner spinner;
  private final Spin spin;

  private final AutonLeft autonLeft;
  private final AutonMiddle autonMiddle;
  private final AutonRight autonRight;
  SendableChooser<Command> chooser = new SendableChooser<>();

  //Button xButton = new JoystickButton(driverController, Constants.xButtonValue);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverController = new XboxController(Constants.driverControllerPort);
    operatorController = new XboxController(Constants.operatorControllerPort);

    driveTrain = new DriveTrain();
    driveGTA = new DriveGTA(driveTrain);
    driveGTA.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveGTA);

    /*
    //If we ever do mecanum, mecanumDriveTrain needs to be set to the default drivetrain or something like that
    //Else there might be an driver station error that says mecanum drive output isnt updated often enough
    mecanumDriveTrain = new MecanumDriveTrain();
    driveMecanum = new DriveMecanum(mecanumDriveTrain);
    driveMecanum.addRequirements(mecanumDriveTrain);
    mecanumDriveTrain.setDefaultCommand(driveMecanum);
    */

    elevator = new Elevator();
    liftUpAndDown = new LiftUpAndDown(elevator);
    liftUpAndDown.addRequirements(elevator);
    elevator.setDefaultCommand(liftUpAndDown);
    timedLift = new TimedLift(elevator);
    timedLift.addRequirements(elevator);

    move = new Move(driveTrain);
    move.addRequirements(driveTrain);

    limelight = new Limelight();
    targetX = new TargetX(driveTrain);
    targetX.addRequirements(driveTrain);
    targetX.addRequirements(limelight);
    targetDistance = new TargetDistance(driveTrain);
    targetDistance.addRequirements(driveTrain);
    targetDistance.addRequirements(limelight);
    seekLeft = new SeekLeft(driveTrain);
    seekLeft.addRequirements(driveTrain);
    seekLeft.addRequirements(limelight);
    seekRight = new SeekRight(driveTrain);
    seekRight.addRequirements(driveTrain);
    seekRight.addRequirements(limelight);
    targetingSequence = new TargetingSequence(driveTrain);
    targetingSequence.addRequirements(driveTrain);
    targetingSequence.addRequirements(limelight);

    turnRight90 = new TurnRight90(driveTrain);
    turnRight90.addRequirements(driveTrain);
    turnRight902 = new TurnRight902(driveTrain);
    turnRight902.addRequirements(driveTrain);
    turnRight903 = new TurnRight903(driveTrain);
    turnRight903.addRequirements(driveTrain);

    shooter = new Shooter();
    shoot = new Shoot(shooter);
    shoot.addRequirements(shooter);
    timedShoot = new TimedShoot(shooter);
    timedShoot.addRequirements(shooter);

    feeder = new Feeder();
    feed = new Feed(feeder);
    feed.addRequirements(feeder);

    parallelShootAndFeed = new ParallelShootAndFeed(shooter, feeder);
    parallelShootAndFeed.addRequirements(shooter);
    parallelShootAndFeed.addRequirements(feeder);

    shootAndFeed = new ShootAndFeed(shooter, feeder);
    shootAndFeed.addRequirements(shooter);
    shootAndFeed.addRequirements(feeder);
    
    spinner = new Spinner();
    spin = new Spin(spinner);
    spin.addRequirements(spinner);

    intake = new Intake();
    takeInBalls = new TakeInBalls(intake);
    takeInBalls.addRequirements(intake);
    intake.setDefaultCommand(takeInBalls);
    //idk why the new vids put setDefaultCommand for the intake but not the shooter

    //UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
    //camera1.setResolution(Constants.camera1XAxisResolution, Constants.camera1YAxisResolution);
    //new vids(part 7) says we should probably make a whole camera subsystem for better
    //functionality, but this works

    autonLeft = new AutonLeft(driveTrain, shooter, feeder);
    autonMiddle = new AutonMiddle(driveTrain, shooter, feeder);
    autonRight = new AutonRight(driveTrain, shooter, feeder);

    //Add choices as option here
    chooser.addOption("Autonomous Middle", autonMiddle);
    chooser.addOption("Autonomous Right", autonRight);
    //Default option happens if nothing is selected on dashboard
    chooser.setDefaultOption("Autonomous Left", autonLeft);
    //Add choices to SmartDashboard
    SmartDashboard.putData("Autonomous", chooser);
    //maybe change default to just move forward off the line then you get the points. This
    //would only occur if you forgot to select an option before the match

    // Configure the button bindings. Method is defined below
    configureButtonBindings();
  }

  public double getDriverRawAxis(int axis){
    //return driverController.getRawAxis(axis);
    //this error might have something to do with the squared values in DriveGTA
    return 0;
  }

  public double getDriverDeadzoneAxis(int axis){
    //double rawValue = driverController.getRawAxis(axis);
    //return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
    //this error might have something to do with the squared values in DriveGTA
    return 0;
  }

  public double getOperatorDeadzoneAxis(int axis){
    double rawValue = operatorController.getRawAxis(axis);
    return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //either one of these should work bc it's getting the same value, just from different places
    //JoystickButton shootButton = new JoystickButton(operatorController, Constants.shootButton);
    //JoystickButton shootButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    //shootButton.whenPressed(new TimedShoot(shooter));

    //JoystickButton feedButton = new JoystickButton(operatorController, Constants.feedButton);
    //feedButton.whileHeld(new Feed(feeder));

    //JoystickButton shootAndFeedButton = new JoystickButton(operatorController, 
      //Constants.shootAndFeedButton);
    //shootAndFeedButton.whileHeld(new ShootAndFeed(shooter, feeder));

    //JoystickButton limelightTargetButton = new JoystickButton(driverController, 
      //Constants.limelightTargetButton);
    //limelightTargetButton.whileHeld(new TargetX(driveTrain));
    //limelightTargetButton.whileHeld(new TargetDistance(driveTrain));
    //limelightTargetButton.whileHeld(new SeekLeft(driveTrain));
    //limelightTargetButton.whileHeld(new SeekRight(driveTrain));
    //limelightTargetButton.whileHeld(new TargetingSequence(driveTrain));

    JoystickButton moveButton = new JoystickButton(driverController, Constants.moveButton);
    moveButton.whenPressed(new Move(driveTrain, 0.3, -0.3, 1));

    //JoystickButton intakeButton = new JoystickButton(operatorController, Constants.intakeButton);
    //intakeButton.whileHeld(new TakeInBalls(intake));

    //JoystickButton timedLiftButton = new JoystickButton(operatorController, Constants.timedLiftButton);
    //timedLiftButton.whenPressed(new TimedLift(elevator));

    //JoystickButton spinButton = new JoystickButton(operatorController, Constants.spinButton);
    //spinButton.whenPressed(new Spin(spinner));

    JoystickButton turnRight90Button = new JoystickButton(driverController, Constants.turnRight90Button);
    turnRight90Button.whenPressed(new TurnRight90(driveTrain));

    //JoystickButton turnRight902Button = new JoystickButton(driverController, Constants.turnRight902Button);
    //turnRight902Button.whenPressed(new TurnRight902(driveTrain));

    JoystickButton turnRight903Button = new JoystickButton(driverController, Constants.turnRight903Button);
    turnRight903Button.whenPressed(new TurnRight903(driveTrain));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
    //return move;
  }
}
