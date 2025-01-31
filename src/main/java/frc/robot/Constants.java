package frc.robot;

// import com.revrobotics.spark.SparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

import static frc.robot.RobotSpecificConstants.getFrontToBackAxleToAxleMeters;
import static frc.robot.RobotSpecificConstants.getSideToSideTreadCenterToTreadCenterMeters;
import static frc.robot.RobotSpecificConstants.getAngleOffsetModule0;
import static frc.robot.RobotSpecificConstants.getAngleOffsetModule1;
import static frc.robot.RobotSpecificConstants.getAngleOffsetModule2;
import static frc.robot.RobotSpecificConstants.getAngleOffsetModule3;

public final class Constants {

  public static final int DRIVER_XBOX_CONTROLLER_PORT = 0;

  public static final double SPEAKER_SHOOTING_ARM_ANGLE = 72.0;
  public static final double SPEAKER_SHOOTING_POWER = 0.8;
  public static final double FLOOR_INTAKE_ARM_ANGLE = 18.0;
  public static final double STOWED_ARM_ANGLE = 40.0;
  public static final double PODIUM_SHOOTING_ARM_ANGLE = 55.0;
  public static final double AMP_SCORING_ARM_ANGLE = 100;
  public static final double AMP_SCORING_POWER = 0.5;
  
  public static final int SHOOTER_MOTOR_1_ID = 57;
  public static final int SHOOTER_MOTOR_2_ID = 58;

  public static final int INTAKE_MOTOR_1_ID = 55;
  public static final int INTAKE_MOTOR_2_ID = 54;

  public static final int ARM_ROTATION_MOTOR_ID = 56;
  public static final int ARM_ENCODER_CHANNEL = 9; // TODO: need to find an actual ID for this
  
  public static final int CLIMBER_MOTOR_ID = 61;
  // public static final int CLIMBER_RIGHT_ID = 62;

  public static final int GRABBER_LIMIT_SWITCH_CHANNEL = 0; // TODO: need to find an actual ID for this
  
  public static final int USELESS_ARM_MOTOR_ID = 51;
  public static final int GRABBER_MOTOR_ID = 52;
  public static final int USELESS_ARM_CAN_CODER_ID = 50;

  public static final class Swerve {
    public static final double STICK_DEADBAND = 0.06; // Was 0.1

    public static final int PIGEON_ID = 7;
    public static final int POWER_DISTRIBUTION_CENTER = 6;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double FRONT_TO_BACK_AXLE_TO_AXLE_METERS = getFrontToBackAxleToAxleMeters();
    public static final double SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_METERS = getSideToSideTreadCenterToTreadCenterMeters();
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //12.56637

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // original value 6.75
    public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_METERS / 2.0, FRONT_TO_BACK_AXLE_TO_AXLE_METERS / 2.0),
            new Translation2d(SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_METERS / 2.0, -FRONT_TO_BACK_AXLE_TO_AXLE_METERS / 2.0),
            new Translation2d(-SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_METERS / 2.0, FRONT_TO_BACK_AXLE_TO_AXLE_METERS / 2.0),
            new Translation2d(-SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_METERS / 2.0, -FRONT_TO_BACK_AXLE_TO_AXLE_METERS / 2.0));

    /* Swerve Voltage Compensation */
    public static final double VOLTAGE_COMP = 12.0;

    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 20;
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 80;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KFF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 1.0; // original value 0.1 // good manual driving 0.3
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_KS = 0.667;
    public static final double DRIVE_KV = 2.44;
    public static final double DRIVE_KA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_CONVERSION_POSITION_FACTOR =
        (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
    public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = 3.5; // meters per second orginal value 4.5 FIXME: the speed does not seen to be changing
    public static double MAX_ANGULAR_VELOCITY = 2.0;//8.0;//11.5;
    public static double MAX_ANGULAR_VELOCITY_FAST_MODE = 4.0;
    public static double MAX_ANGULAR_VELOCITY_SLOW_MODE = 2.0;
    public static double MAX_ANGULAR_VELOCITY_SUPER_SLOW_MODE = 0.5;

    /* Neutral Modes */
    // public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kBrake;
    // public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean DRIVE_INVERT = true;
    public static final boolean ANGLE_INVERT = true;

    /* Angle Encoder Invert */
    public static final boolean CAN_CODER_INVERT = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class MOD_0 {
      public static final int DRIVE_MOTOR_ID = 32;
      public static final int ANGLE_MOTOR_ID = 31;
      public static final int CAN_CODER_ID = 30;
      public static final Rotation2d ANGLE_OFFSET = getAngleOffsetModule0();
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class MOD_1 {
      public static final int DRIVE_MOTOR_ID = 22;
      public static final int ANGLE_MOTOR_ID = 21;
      public static final int CAN_CODER_ID = 20;
      public static final Rotation2d ANGLE_OFFSET = getAngleOffsetModule1();
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class MOD_2 {
      public static final int DRIVE_MOTOR_ID = 42;
      public static final int ANGLE_MOTOR_ID = 41;
      public static final int CAN_CODER_ID = 40;
      public static final Rotation2d ANGLE_OFFSET = getAngleOffsetModule2();
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class MOD_3 {
      public static final int DRIVE_MOTOR_ID = 12;
      public static final int ANGLE_MOTOR_ID = 11;
      public static final int CAN_CODER_ID = 10;
      public static final Rotation2d ANGLE_OFFSET = getAngleOffsetModule3();
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3.5; // original value 3
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5; // original value 3
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static  double PX_CONTROLLER = 1.0;
    public static  double IX_CONTROLLER = 0.0;
    public static  double DX_CONTROLLER = 0.0;

    public static  double PY_CONTROLLER = 0.0;
    public static  double IY_CONTROLLER = 0.0;
    public static  double DY_CONTROLLER = 0.0;
    
    public static  double P_THETA_CONTROLLER = 0.0;
    public static  double I_THETA_CONTROLLER = 0.0;
    public static  double D_THETA_CONTROLLER = 0.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

}
