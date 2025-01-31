package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkPIDController;
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

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

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
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
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
    angleMotor.restoreFactoryDefaults();
    SparkMaxUtil.setSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);
    angleMotor.setInverted(Constants.Swerve.ANGLE_INVERT);
    // angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);

    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);

    angleController.setP(Constants.Swerve.ANGLE_KP);
    angleController.setI(Constants.Swerve.ANGLE_KI);
    angleController.setD(Constants.Swerve.ANGLE_KD);
    angleController.setFF(Constants.Swerve.ANGLE_KFF);

    angleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    angleMotor.burnFlash();
    
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT);
    driveMotor.setInverted(Constants.Swerve.DRIVE_INVERT);
    // driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);

    driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
    // SmartDashboard.putNumber("Mod " + moduleNumber + " velocity conversion factor actual", driveEncoder.getVelocityConversionFactor());
    // SmartDashboard.putNumber("Mod " + moduleNumber + " velocity conversion factor desired", Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);

    driveController.setP(Constants.Swerve.DRIVE_KP);
    driveController.setI(Constants.Swerve.DRIVE_KI);
    driveController.setD(Constants.Swerve.DRIVE_KD);
    driveController.setFF(Constants.Swerve.DRIVE_KFF);

    driveController.setP(10, 1);  // NORMAL
    driveController.setI(0, 1);
    driveController.setD(0, 1);
    driveController.setFF(0, 1);
    driveController.setOutputRange(-0.25, 0.25, 1);  // FIXME: Bump this to be a tad higher?

    driveController.setP(10, 2);  // FAST
    driveController.setI(0, 2);
    driveController.setD(0, 2);
    driveController.setFF(0, 2);
    driveController.setOutputRange(-0.5, 0.5, 2);

    driveController.setP(10, 3);  // SLOW
    driveController.setI(0, 3);
    driveController.setD(0, 3);
    driveController.setFF(0, 3);
    driveController.setOutputRange(-0.2, 0.2, 3);

    driveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    driveMotor.burnFlash();
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
          0,
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
      ControlType.kPosition, pidSlot);  // NOTE: Have 0..3 pid controller positions if we choose to use them
  }

  public void setSpeedPercent(double percent){
    driveMotor.set(percent);
  }

  public void goToPositionMeters(double desiredPositionMeters, int pidSlot) {
    driveController.setReference(
      desiredPositionMeters, // in to meters is / 39.37
      ControlType.kPosition, pidSlot);  // NOTE: Have 0..3 pid controller positions if we choose to use them
      // in to meters is / 39.37
  }

  // public void goToPositionInches(double desiredPositionInches, int pidSlot) {
  //   driveController.setReference(desiredPositionInches / 39.37, ControlType.kPosition, pidSlot);
  // }

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

  public void resetDriveEncoder(){
    driveEncoder.setPosition(0.0);
  }

  public void burnFlash(){
    driveMotor.burnFlash();
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

  public double countsPerRev(){
    return driveEncoder.getCountsPerRevolution();
  }
}
