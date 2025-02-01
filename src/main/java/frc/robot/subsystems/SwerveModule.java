package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.SparkMaxUtil;
import frc.lib.util.SparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

@SuppressWarnings(value = { "removal" })
public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void setDesiredPosition(SwerveModulePosition desiredPosition) {
    setAngle(desiredPosition);
    setPosition(desiredPosition);
  }

  public void setDesiredPosition(SwerveModulePosition desiredPosition, int pidSlot) {
    setAngle(desiredPosition);
    setPosition(desiredPosition, pidSlot);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
    lastAngle = Rotation2d.fromDegrees(absolutePosition);
    // System.out.println("resetToAbsolute called: " + absolutePosition);
  }

  public void resetRelativeEncoders() {
    //integratedAngleEncoder.setPosition(0.0);
    resetDriveEncoder();
    //lastAngle = getState().angle;
    resetToAbsolute();
  }

  public void setWheelToForward() {
    resetToAbsolute();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotorConfig
      .inverted(Constants.Swerve.ANGLE_INVERT)
      .smartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT)
      .voltageCompensation(Constants.Swerve.VOLTAGE_COMP);

    angleMotorConfig.encoder
      .positionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);

    angleMotorConfig.closedLoop
      .p(Constants.Swerve.ANGLE_KP)
      .i(Constants.Swerve.ANGLE_KI)
      .d(Constants.Swerve.ANGLE_KD)
      .velocityFF(Constants.Swerve.ANGLE_KFF);

    angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxUtil.setSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
  
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotorConfig
      .inverted(Constants.Swerve.DRIVE_INVERT)
      .smartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT)
      .voltageCompensation(Constants.Swerve.VOLTAGE_COMP);

    driveMotorConfig.encoder
      .velocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR)
      .positionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);

    driveMotorConfig.closedLoop
      .p(Constants.Swerve.DRIVE_KP)
      .i(Constants.Swerve.DRIVE_KI)
      .d(Constants.Swerve.DRIVE_KD)
      .velocityFF(Constants.Swerve.DRIVE_KFF);
    
    driveMotorConfig.closedLoop  // NORMAL
      .pidf(10, 0, 0, 0, ClosedLoopSlot.kSlot1)
      .outputRange(-0.25, 0.25, ClosedLoopSlot.kSlot1);

    driveMotorConfig.closedLoop  // FAST
      .pidf(10, 0, 0, 0, ClosedLoopSlot.kSlot2)
      .outputRange(-0.5, 0.5, ClosedLoopSlot.kSlot2);

    driveMotorConfig.closedLoop  // SLOW
      .pidf(10, 0, 0, 0, ClosedLoopSlot.kSlot3)
      .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot3);

    SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);

    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
      // SmartDashboard.putNumber("Mod " + moduleNumber + " speed", desiredState.speedMetersPerSecond);
      // SmartDashboard.putNumber("Mod " + moduleNumber + " % val", percentOutput);
      driveMotor.set(percentOutput);
    } else {
      // SmartDashboard.putNumber("Auto " + moduleNumber + " speed", desiredState.speedMetersPerSecond);
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          0.0); //feedforward.calculate(desiredState.speedMetersPerSecond)
    }
  }

  public void stop() {
    driveMotor.set(0.0);
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void setAngle(SwerveModulePosition desiredPosition) {
    setAngle(desiredPosition.angle);
  }

  public void setAngle(Rotation2d desiredAngle) {
    angleController.setReference(desiredAngle.getDegrees(), ControlType.kPosition);
    lastAngle = desiredAngle;
  }

  private void setPosition(SwerveModulePosition desiredPosition) {
    driveController.setReference(
      desiredPosition.distanceMeters,
      ControlType.kPosition);  // NOTE: Have 0..3 pid controller positions if we choose to use them
  }

  private void setPosition(SwerveModulePosition desiredPosition, int pidSlot) {
    driveController.setReference(
      desiredPosition.distanceMeters,
      ControlType.kPosition, convertPidSlotToClosedLoopSlot(pidSlot));  // NOTE: Have 0..3 pid controller positions if we choose to use them
  }

  public void setSpeedPercent(double percent){
    driveMotor.set(percent);
  }

  public void goToPositionMeters(double desiredPositionMeters, int pidSlot) {
    driveController.setReference(
      desiredPositionMeters, // in to meters is / 39.37
      ControlType.kPosition, convertPidSlotToClosedLoopSlot(pidSlot));  // NOTE: Have 0..3 pid controller positions if we choose to use them
      // in to meters is / 39.37
  }

  // public void goToPositionInches(double desiredPositionInches, int pidSlot) {
  //   driveController.setReference(desiredPositionInches / 39.37, ControlType.kPosition, pidSlot);
  // }

// FIXME: Need to configure differently now
/*
  public void setP(double p, int pidSlot){
    driveController.setP(p, pidSlot);
  }
  
  public void setI(double i, int pidSlot){
    driveController.setI(i, pidSlot);
  }

  public void setD(double d, int pidSlot){
    driveController.setD(d, pidSlot);
  }

  public void setMinMax(double min, double max, int pidSlot){
    driveController.setOutputRange(min, max, pidSlot);
  }
*/
  public void resetDriveEncoder(){
    driveEncoder.setPosition(0.0);
  }

  public void burnFlash(){
    // FIXME: Burn Flash is a completely separate thing now
    //driveMotor.burnFlash();
  }

  public double getAngleOffset() {
    return angleOffset.getDegrees();
  }

  public double getDesiredAngleAsDegrees() {
    return lastAngle.getDegrees();
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    // NOTE: the driveEncoder.getPosition() needs to be in meters, so ensure conversion factor does that
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public double getDriveEncoder(){
    return driveEncoder.getPosition();
  }

  // public double countsPerRev(){
  //   return driveEncoder.getCountsPerRevolution();
  // }

  private static ClosedLoopSlot convertPidSlotToClosedLoopSlot(int pidSlot) {
    switch (pidSlot) {
      case(0): return ClosedLoopSlot.kSlot0;
      case(1): return ClosedLoopSlot.kSlot1;
      case(2): return ClosedLoopSlot.kSlot2;
      case(3): return ClosedLoopSlot.kSlot3;
      default: return ClosedLoopSlot.kSlot0;
    }
  }
}
