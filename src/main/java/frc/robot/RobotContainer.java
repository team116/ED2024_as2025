// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.autos.*;
import frc.robot.autos.primitives.IntakeCommand;
import frc.robot.autos.primitives.MoveArmToAngle;
import frc.robot.autos.primitives.OutTakeCommand;
import frc.robot.autos.primitives.PrepNoteToShoot;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection;
import frc.robot.autos.primitives.RunShooterAtPower;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection.RollerDirection;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings(value = { "removal" })
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick gunnerStation = new Joystick(1);
  private final Joystick gunnerLogitech = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyroReverse =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final JoystickButton driverOutakeButton =
       new JoystickButton(driver, XboxController.Button.kX.value);

  // private final JoystickButton armMotorForward =
  //     new JoystickButton(driver, XboxController.Button.kY.value);

  // private final JoystickButton armMotorReverse =
  //     new JoystickButton(driver, XboxController.Button.kA.value);

  // private final JoystickButton enableArmLimitSwitches =
  //     new JoystickButton(gunnerStation, 4);

  private final JoystickButton autoAlignMacroButton =
      new JoystickButton(driver, XboxController.Button.kB.value);

  private final JoystickButton resetAngleEncodersButton =
      new JoystickButton(driver, XboxController.Button.kY.value);

  private final JoystickButton shooterWheelsSpinUpButton = new JoystickButton(gunnerLogitech, 1);

  private final JoystickButton gunnerIntakeThenPrepNoteButton = new JoystickButton(gunnerLogitech, 2);
  private final JoystickButton gunnerIntakeButton = new JoystickButton(gunnerLogitech, 5);
  private final JoystickButton gunnerOutTakeButton = new JoystickButton(gunnerLogitech, 6);
  private final JoystickButton gunnerPrepNoteToShoot = new JoystickButton(gunnerLogitech, 11);
  private final JoystickButton gunnerAmpScoringButton = new JoystickButton(gunnerLogitech,3);
  private final JoystickButton gunnerSpeakerScoringButton = new JoystickButton(gunnerLogitech,4);


  private final JoystickButton gunnerArmUpSlowButton = new JoystickButton(gunnerLogitech, 9);
  private final JoystickButton gunnerArmDownSlowButton = new JoystickButton(gunnerLogitech, 10);
  private final JoystickButton gunnerArmToCloseSpeakerPositionButton = new JoystickButton(gunnerLogitech,12);
  
  private final JoystickButton gunnerPadArmToIntakePositionButton = new JoystickButton(gunnerStation, 5);
  private final JoystickButton gunnerPadArmToIntakePositionButton2 = new JoystickButton(gunnerStation, 7);
  private final JoystickButton gunnerPadArmToStowedPositionButton = new JoystickButton(gunnerStation, 6);
  private final JoystickButton gunnerPadArmToStowedPositionButton2 = new JoystickButton(gunnerStation, 8);
  private final JoystickButton gunnerPadArmToPodiumSpeakerPositionButton = new JoystickButton(gunnerStation, 4);
  private final JoystickButton gunnerPadArmToCloseSpeakerPositionButton = new JoystickButton(gunnerStation, 11);
  private final JoystickButton gunnerPadArmToAmpPositionButton = new JoystickButton(gunnerStation, 12);
  


  private final JoystickButton gunnerClimberUpButton = new JoystickButton(gunnerLogitech, 8);
  private final JoystickButton gunnerClimberUpSlowButton  = new JoystickButton(gunnerLogitech, 7);

  private final CommandXboxController driverXBoxController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_PORT);

  private final Trigger driverLeftTrigger = driverXBoxController.leftTrigger(0.5);
  private final Trigger driverRightTrigger = driverXBoxController.rightTrigger(0.5);

  private final POVButton dpadUp = new POVButton(driver, 0);
  // private final POVButton dpadRight = new POVButton(driver, 90);
  private final POVButton dpadDown = new POVButton(driver, 180);
  // private final POVButton dpadLeft = new POVButton(driver, 270);

   /* Subsystems */
  // private final UselessArm arm = new UselessArm(); // don't need these right now
  // private final SemiUsefulGrabber grabber = new SemiUsefulGrabber();
  // private final UselessLeds leds = new UselessLeds();

  private final Limelight limelight = new Limelight();
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final Shooter shooter = new Shooter();
  private final Arm arm = new Arm();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final Climber climber = new Climber();
  
  private final Pigeon2 gyro = s_Swerve.getGyro();

  private final SendableChooser<Command> sendableChooser;

  private final ToggleStateBooleanSupplier robotCentricState = new ToggleStateBooleanSupplier();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SmartDashboard.putNumber("Delay", 0);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> shape(-driver.getRawAxis(translationAxis)),
            () -> shape(-driver.getRawAxis(strafeAxis)),
            () -> rotationShape(-driver.getRawAxis(rotationAxis)),
            () -> robotCentricState.getAsBoolean()));

    // limelight.setDefaultCommand(new DefaultLimelightCommand(limelight));

    //arm.setDefaultCommand(new DefaultArmCommand(arm, gunnerLogitech, gunnerStation));
    // arm.setDefaultCommand(new DefaultArmCommand(arm, gunnerLogitech, gunnerStation));

    //NamedCommands.registerCommand("First Command", new InstantCommand(() -> SmartDashboard.putString("The Event Marker has been pressed", "It has been pressed")));

   

    //grabber.setDefaultCommand(new GrabberCommand(grabber));

    //leds.setDefaultCommand(new DefaultLedCommand(leds, gunnerLogitech, gunnerStation));

    // Configure the button bindings
    configureButtonBindings();


    double delay = SmartDashboard.getNumber("Delay", 0);
    // sendableChooser = AutoBuilder.buildAutoChooser();
    sendableChooser = new SendableChooser<>();
    sendableChooser.addOption("Do Nothing", new DoNothingCommand());

    sendableChooser.addOption("Center One Note Auto", new OneNoteAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Center Two Note Auto", new TwoNoteAuto(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Red Amp Two Note Auto", new RedAmpSideTwoNote(s_Swerve, arm, intakeSubsystem, shooter));
    sendableChooser.addOption("Blue Amp Two Note Auto", new BlueAmpSideTwoNote(s_Swerve, arm, intakeSubsystem, shooter));

    sendableChooser.addOption("Red Amp One Note Auto", new OneNoteLeftAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Red Source One Note Auto", new OneNoteRightAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Red Source Shallow One Note Auto", new OneNoteShallowRightAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));

    sendableChooser.addOption("Blue Amp One Note Auto", new OneNoteRightAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Blue Source One Note Auto", new OneNoteLeftAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));
    sendableChooser.addOption("Blue Source Shallow One Note Auto", new OneNoteShallowLeftAutoAndMoveOut(s_Swerve, arm, intakeSubsystem, shooter, delay));

    sendableChooser.addOption("drive back", new DriveBackwards(s_Swerve));
    SmartDashboard.putData("Auto Mode", sendableChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyroReverse.onTrue(new InstantCommand(() -> s_Swerve.reverseZeroGyro()));
    driverLeftTrigger.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // toggleTesterButton.onTrue(new InstantCommand(() -> limelight.toggleStreamMode()));
    autoAlignMacroButton.onTrue(new AprilTagAlignmentCommand(s_Swerve, limelight));

    resetAngleEncodersButton.onTrue(new InstantCommand(() -> s_Swerve.resetAngleEncoders()));

    shooterWheelsSpinUpButton.whileTrue(new RunShooterAtPower(shooter, gunnerLogitech));

    driverRightTrigger.onTrue(new InstantCommand(() -> s_Swerve.toggleSlowMode()));
    dpadDown.onTrue(new InstantCommand(() -> s_Swerve.toggleSuperSlowMode()));

    robotCentric.onTrue(new InstantCommand(() -> robotCentricState.toggleState()));

    gunnerArmUpSlowButton.onTrue(new InstantCommand(() -> arm.moveUp()));
    gunnerArmDownSlowButton.onTrue(new InstantCommand(() -> arm.moveDown()));
    gunnerArmUpSlowButton.onFalse(new InstantCommand(() -> arm.stop()));
    gunnerArmDownSlowButton.onFalse(new InstantCommand(() -> arm.stop()));
    gunnerArmToCloseSpeakerPositionButton.whileTrue(new MoveArmToAngle(arm, Constants.SPEAKER_SHOOTING_ARM_ANGLE, 3.0));
    
    gunnerPadArmToIntakePositionButton.whileTrue(new MoveArmToAngle(arm, Constants.FLOOR_INTAKE_ARM_ANGLE, 3.0));
    gunnerPadArmToStowedPositionButton.whileTrue(new MoveArmToAngle(arm, Constants.STOWED_ARM_ANGLE, 3.0));
    gunnerPadArmToIntakePositionButton2.whileTrue(new MoveArmToAngle(arm, Constants.FLOOR_INTAKE_ARM_ANGLE, 3.0));
    gunnerPadArmToStowedPositionButton2.whileTrue(new MoveArmToAngle(arm, Constants.STOWED_ARM_ANGLE, 3.0));

    gunnerPadArmToPodiumSpeakerPositionButton.whileTrue(new MoveArmToAngle(arm, Constants.PODIUM_SHOOTING_ARM_ANGLE, 3.0));
    gunnerPadArmToCloseSpeakerPositionButton.whileTrue(new MoveArmToAngle(arm, Constants.SPEAKER_SHOOTING_ARM_ANGLE, 3.0));
    gunnerPadArmToAmpPositionButton.whileTrue(new MoveArmToAngle(arm, Constants.AMP_SCORING_ARM_ANGLE, 3.0));



    gunnerClimberUpButton.onTrue(new InstantCommand(() -> climber.pullUp()));
    gunnerClimberUpSlowButton.onTrue(new InstantCommand(() -> climber.pullUpSlow()));
    gunnerClimberUpButton.onFalse(new InstantCommand(() -> climber.stop()));
    gunnerClimberUpSlowButton.onFalse(new InstantCommand(() -> climber.stop()));

    gunnerIntakeButton.whileTrue(new IntakeCommand(intakeSubsystem, Double.POSITIVE_INFINITY));
    gunnerOutTakeButton.whileTrue(new OutTakeCommand(intakeSubsystem, Double.POSITIVE_INFINITY));
    gunnerPrepNoteToShoot.onTrue(new PrepNoteToShoot(intakeSubsystem));
    gunnerIntakeThenPrepNoteButton.onTrue(new IntakeCommand(intakeSubsystem, Double.POSITIVE_INFINITY));
    gunnerIntakeThenPrepNoteButton.onFalse(new PrepNoteToShoot(intakeSubsystem));
    gunnerAmpScoringButton.whileTrue(new AmpScoring(shooter, intakeSubsystem, arm));
    gunnerSpeakerScoringButton.whileTrue(new SpeakerScoringClose(shooter, intakeSubsystem, arm));
    // XBox
    /*

    Controller:
    Left stick = drive
    Right stick left/right = turn left/right
    Left trigger = Reset how the robot thinks the field is rotated relative to it
    Right trigger = Toggle slow mode
    Right bumper = Toggle field/robot centric driving
    Dpad down = Toggle super slow mode. WARNING: Super slow mode is by default ON.
    
    Left bumper = Broken flip fowards/backwards


    //OLD:

    // Y for slow mode
    // Right trigger - inwards
    // Left trigger - outwards

    // Logitech
    // Raw 1 - Trigger - Stowed in robot ()
    // Raw 2 - Human Intake

    // y-axis -> "up"/"down"   (invert, as negative)

    // Raw 7  - Cone High
    // Raw 8  - Cube High
    // Raw 9  - Cone Mid
    // Raw 10 - Cube Mid
    // Raw 11 - Low Score
    // Raw 12 - Floor Intake 


    // Gunner
    // Raw 1 - Intake
    // Raw 2 - Outake

    // Raw 3 - Claw Limit Switch Disable (if "on")
    // Raw 4 - Arm Limit Switch Disable (if "on")
    // Raw 5 - Disable all arm operations

    // Raw 7 - Cube/Color Switch  (RGB somewhere)
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new exampleAuto(s_Swerve);
    return sendableChooser.getSelected();
    //return new DoNothingCommand();
  }

  public static double shape(double start){
    if (start < 0){
      return -(start * start);
    }
    return start * start;
  }

  public static double rotationShape(double start) {
    //return (start * start * start) /2.0 ;  // was original dividing by 2 */ / 2.0d;
    return shape(start);
  }

  public double getDistanceFromAprilTagInches() {
    return limelight.getDistanceFromAprilTagInches();
  }

  public double getShooter1MotorSpeed() {
    return shooter.getMotor1Velocity();
  }

  public double getShooter2MotorSpeed() {
    return shooter.getMotor2Velocity();
  }

  public double getArmAngle() {
    return arm.getAngleDegrees();
  }

  public void resetRobotToCorrectAutonomousFieldPosition() {
    s_Swerve.zeroGyro();
  }

  public double getManualShooterSpeed() {
    return -gunnerLogitech.getThrottle();
  }

  public void enableLeds() {
    // leds.enable();
  }

  public void disableLeds() {
    // leds.disable();
  }

  public boolean getRobotCentricState() {
    return robotCentricState.getAsBoolean();
  }
}
