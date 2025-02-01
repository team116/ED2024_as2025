package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
// FIXME: PathPlanner totally changed, no need for just testing out running code since not used in 2025
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;;

@SuppressWarnings(value = { "removal" })
public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private boolean inSlowMode = false;
  private boolean superSlowMode = false;

  public SwerveSubsystem() {
    gyro = new Pigeon2(Swerve.PIGEON_ID);
    // FIXME: Not sure what best equivalent is at this point
    //gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Swerve.MOD_0.CONSTANTS),
          new SwerveModule(1, Swerve.MOD_1.CONSTANTS),
          new SwerveModule(2, Swerve.MOD_2.CONSTANTS),
          new SwerveModule(3, Swerve.MOD_3.CONSTANTS)
        };

    swerveOdometry = new SwerveDriveOdometry(Swerve.SWERVE_KINEMATICS, getYaw(), getPositions());

    // AutoBuilder.configureHolonomic(
    //             this::getPose, 
    //             this::resetOdometry, 
    //             this::getChassisSpeeds, 
    //             this::driveAuto, 
    //             new HolonomicPathFollowerConfig(
    //               new PIDConstants(AutoConstants.PX_CONTROLLER, AutoConstants.IX_CONTROLLER, AutoConstants.DX_CONTROLLER), 
    //               new PIDConstants(AutoConstants.P_THETA_CONTROLLER, AutoConstants.I_THETA_CONTROLLER, AutoConstants.D_THETA_CONTROLLER),
    //               3.5, // Original value 4.6
    //               0.4318, 
    //               new ReplanningConfig()
    //               ), 
    //               () -> {
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                   return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //               }, 
    //               this);

    field = new Field2d();
    // SmartDashboard.putData("Field", field);
  }

  public void reconfigureAutoBuilder() {
    // AutoBuilder.configureHolonomic(
    //             this::getPose, 
    //             this::resetOdometry, 
    //             this::getChassisSpeeds, 
    //             this::driveAuto, 
    //             new HolonomicPathFollowerConfig(
    //               new PIDConstants(AutoConstants.PX_CONTROLLER, AutoConstants.IX_CONTROLLER, AutoConstants.DX_CONTROLLER), 
    //               new PIDConstants(AutoConstants.P_THETA_CONTROLLER, AutoConstants.I_THETA_CONTROLLER, AutoConstants.D_THETA_CONTROLLER),
    //               2.0, // Original value 4.6
    //               0.4318, 
    //               new ReplanningConfig()
    //               ), 
    //               () -> {
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                   return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //               }, 
    //               this);
  }

  public void setAutoConstant(String name, double value) {
    switch (name) {
      case "PX":
        setAutoPX(value);
        return;
      case "IX":
        setAutoIX(value);
        return;
      case "DX":
        setAutoDX(value);
        return;
      case "PY":
        setAutoPY(value);
        return;
      case "IY":
        setAutoIY(value);
        return;
      case "DY":
        setAutoDY(value);
        return;
      case "PTheta":
        setAutoPTheta(value);
        return;
      case "ITheta":
        setAutoITheta(value);
        return;
      case "DTheta":
        setAutoDTheta(value);
        return;
      default:
        throw new IllegalArgumentException("Not one of the constants");
    }
  }

  public double getAutoConstant(String name) {
    switch (name) {
      case "PX":
        return AutoConstants.PX_CONTROLLER;
      case "IX":
        return AutoConstants.IX_CONTROLLER;
      case "DX":
        return AutoConstants.DX_CONTROLLER;
      case "PY":
        return AutoConstants.PY_CONTROLLER;
      case "IY":
        return AutoConstants.IY_CONTROLLER;
      case "DY":
        return AutoConstants.DY_CONTROLLER;
      case "PTheta":
        return AutoConstants.P_THETA_CONTROLLER;
      case "ITheta":
        return AutoConstants.I_THETA_CONTROLLER;
      case "DTheta":
        return AutoConstants.D_THETA_CONTROLLER;
      default:
        throw new IllegalArgumentException("Not one of the constants");
    }
  }

  public void setAutoPX(double value) {
    AutoConstants.PX_CONTROLLER = value;
  }

  public void setAutoIX(double value) {
    AutoConstants.IX_CONTROLLER = value;
  }

  public void setAutoDX(double value) {
    AutoConstants.DX_CONTROLLER = value;
  }

  public void setAutoPY(double value) {
    AutoConstants.PY_CONTROLLER = value;
  }

  public void setAutoIY(double value) {
    AutoConstants.IY_CONTROLLER = value;
  }

  public void setAutoDY(double value) {
    AutoConstants.DY_CONTROLLER = value;
  }

  public void setAutoPTheta(double value) {
    AutoConstants.P_THETA_CONTROLLER = value;
  }

  public void setAutoITheta(double value) {
    AutoConstants.I_THETA_CONTROLLER = value;
  }

  public void setAutoDTheta(double value) {
    AutoConstants.D_THETA_CONTROLLER = value;
  }

  public void resetRelativeEncoders() {
    for (SwerveModule swerveModule : mSwerveMods) {
      swerveModule.resetRelativeEncoders();
    }
  }

  public void resetAngleEncoders() {
    for (SwerveModule swerveModule : mSwerveMods) {
      swerveModule.resetToAbsolute();
    }
  }

  public void resetDriveEncoders() {
    for (SwerveModule swerveModule : mSwerveMods) {
      swerveModule.resetDriveEncoder();
    }
  }

  public void setMinMax(double min, double max, int pidSlot){
    for (SwerveModule swerveModule : mSwerveMods){
      //swerveModule.setMinMax(min, max, pidSlot);
    }
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveAuto(ChassisSpeeds speed) {
    SwerveModuleState [] swerveModuleState = Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speed);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, Swerve.MAX_SPEED);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleState [mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      
    }
  }

  public void setModulePositions(SwerveModulePosition[] desiredPositions) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredPosition(desiredPositions[mod.moduleNumber]);
    }
  }

  public void setModulePositions(SwerveModulePosition[] desiredPositions, int pidSlot) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredPosition(desiredPositions[mod.moduleNumber], pidSlot);
    }
  }

  public void stop() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  public void setPID(double p, double i, double d, int pidSlot){
    // FIXME: Nobody cares, as there is NO ARM on the robot (code should have been nuked in 2024)
    // for (SwerveModule mod : mSwerveMods){
    //   mod.setP(p, pidSlot);
    //   mod.setI(i, pidSlot);
    //   mod.setD(d, pidSlot);
    // }
  }

  public void setSpeedPercent(double percent){
    for (SwerveModule mod : mSwerveMods){
      mod.setSpeedPercent(percent);
    }
  }

  public void burnFlash(){
    for (SwerveModule mod : mSwerveMods){
      mod.burnFlash();
    }
  }

  public void runToPositionInMeters(double position, int pidSlot){
    for (SwerveModule mod : mSwerveMods) {
      mod.goToPositionMeters(position, pidSlot);
    }
  }

  // public void runToPositionInInches(double position, int pidSlot) {
  //   for (SwerveModule mod: mSwerveMods) {
  //     mod.goToPositionInches(position, pidSlot);
  //   }
  // }

  public void turnWheelsToAngles(Rotation2d[] angles) {
    mSwerveMods[0].setAngle(angles[0]);
    mSwerveMods[1].setAngle(angles[1]);
    mSwerveMods[2].setAngle(angles[2]);
    mSwerveMods[3].setAngle(angles[3]);
  }

  public void turnWheelsToToAngle(Rotation2d angle) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setAngle(angle);
    }
  }

  public Pigeon2 getGyro(){
    return gyro;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    //swerveOdometry.resetPosition(pose, getYaw());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getStates());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod: mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();  // NOTE: position is already turned into meters from conversion factor
    }
    return positions;
  }

  // NOTE: If robot is facing our alliance wall, call this to reset field orientation.
  public void reverseZeroGyro() {
    gyro.setYaw(180.0);
  }

  public void zeroGyro() {
    gyro.setYaw(0.0);
  }

  public double getPitch(){
    return gyro.getPitch().getValueAsDouble();
  }

  public double getRoll(){
    return gyro.getRoll().getValueAsDouble();
  }

  public Rotation2d getYaw() {
    StatusSignal<Angle> yawStatusSignal = gyro.getYaw();
    return (Swerve.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360.0 - yawStatusSignal.getValueAsDouble())
        : Rotation2d.fromDegrees(yawStatusSignal.getValueAsDouble());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    // swerveOdometry.update(getYaw(), getStates());
    field.setRobotPose(getPose());

    SmartDashboard.putString("Pose ", getPose().toString());
    for (SwerveModule mod : mSwerveMods) {
      String modName = "Mod " + mod.moduleNumber;
      SmartDashboard.putNumber(
          modName + " Cancoder", mod.getCanCoder().getDegrees());
      // SmartDashboard.putNumber(
      //     modName + " Integrated", mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      //     modName + " Velocity", mod.getState().speedMetersPerSecond);
      // SmartDashboard.putNumber(
      //   modName + " Desired", mod.getDesiredAngleAsDegrees());
      // SmartDashboard.putNumber(
      //   modName + " Adj Cancoder", mod.getCanCoder().getDegrees() - mod.getAngleOffset());
      // SmartDashboard.putNumber(modName + " distance Meters", mod.getPosition().distanceMeters);
      // SmartDashboard.putNumber(modName + " drive motor angle", mod.getPosition().angle.getDegrees());
      // SmartDashboard.putNumber(modName + " drive motor encoder", mod.getDriveEncoder());
    }
  }

  public boolean inSlowMode() {
    return inSlowMode;
  }

  public void setSlowMode(boolean newSlowModeValue) {
    inSlowMode = newSlowModeValue;
  }

  public void toggleSlowMode() {
    inSlowMode = !inSlowMode;
  }

  public void toggleSuperSlowMode() {
    superSlowMode = !superSlowMode;
  }

  public void setSuperSlowMode(boolean newSuperSlowMode) {
    superSlowMode = newSuperSlowMode;
  }

  public boolean inSuperSlowMode() {
    return superSlowMode;
  }
}
