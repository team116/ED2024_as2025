package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagAlignmentCommand extends Command {
    private SwerveSubsystem swerve;
    private Limelight limelight;
    private int stabilizedCount;
    private double startTime;
    private double desiredDistanceFromAprilTagInches = 60;

    public AprilTagAlignmentCommand(SwerveSubsystem swerveSubsystem, Limelight limelightSubsystem) {
        this.swerve = swerveSubsystem;
        this.limelight = limelightSubsystem;
        addRequirements(swerveSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        stabilizedCount = 0;
        limelight.ledOn();  // NOTE: Not sure what the delay is between asking to turn on and it being on
    }

    @Override
    public void execute() {
        setDesiredDistanceFromAprilTagInches(SmartDashboard.getNumber("Desired Distance From April Tag Inches", desiredDistanceFromAprilTagInches));
        
        SmartDashboard.putNumber("Desired Distance From April Tag Inches", desiredDistanceFromAprilTagInches);
        SmartDashboard.putNumber("Distance From April Tag in Feet", limelight.getDistanceFromAprilTagFeet());
        SmartDashboard.putNumber("Distance From April Tag Inches", limelight.getDistanceFromAprilTagInches());
        SmartDashboard.putNumber("ta", limelight.getTa());
        SmartDashboard.putNumber("tx", limelight.getTx());
        SmartDashboard.putNumber("ty", limelight.getTy());
        
    
        double offsetAngleDegrees = limelight.horizontalOffsetFromCrosshairAsDegrees();
        double actualDistanceFromAprilTagInches = limelight.getDistanceFromAprilTagInches();
        double distancedLeftToTravelInches = Math.abs(desiredDistanceFromAprilTagInches - actualDistanceFromAprilTagInches);

        if (stillNeedToRotate(offsetAngleDegrees) || stillNeedToMove(desiredDistanceFromAprilTagInches, actualDistanceFromAprilTagInches)) {
                stabilizedCount = 0;
                double degreesPerSecondSpeed = 0.125d;
                double metersPerSecondSpeed = 0.125d;
                if (stillNeedToRotate(offsetAngleDegrees)){
                    if (offsetAngleDegrees > 0.0d) {
                        degreesPerSecondSpeed = -degreesPerSecondSpeed;
                    }
                } else {
                    degreesPerSecondSpeed = 0.0d;
                }
                if (stillNeedToMove(desiredDistanceFromAprilTagInches, actualDistanceFromAprilTagInches)) {
                    if (distancedLeftToTravelInches > 120) {
                        metersPerSecondSpeed = 1.0d;
                    } else if (distancedLeftToTravelInches > 90) {
                        metersPerSecondSpeed = 0.75d;
                    } else if (distancedLeftToTravelInches > 60) {
                        metersPerSecondSpeed = 0.5d;
                    } else if (distancedLeftToTravelInches > 30) {
                        metersPerSecondSpeed = 0.25;
                    } else if (distancedLeftToTravelInches > 10) {
                        metersPerSecondSpeed = 0.125d;
                    }
                    if (needToMoveBackward(desiredDistanceFromAprilTagInches)) {
                        metersPerSecondSpeed = -metersPerSecondSpeed;
                    }
                } else {
                    metersPerSecondSpeed = 0.0d;
                }
                swerve.drive(new Translation2d(metersPerSecondSpeed, 0.0d), degreesPerSecondSpeed, false, true);
        } else {
            swerve.drive(new Translation2d(0.0d, 0.0d), 0, false, true);
            ++stabilizedCount;
        }
    }

    @Override
    public boolean isFinished() {
          // Only exit after 3 non-movements, or timer hits half second
        return (stabilizedCount > 20 || (Timer.getFPGATimestamp() - startTime > 10d));
    }

    @Override
    public void end(boolean interrupted) {
        limelight.ledOff();
    }

    private boolean stillNeedToRotate(double offsetAngleDegrees) {
        return limelight.hasValidTarget() &&
               Math.abs(limelight.horizontalOffsetFromCrosshairAsDegrees()) > 0.75d;
    }

    private boolean stillNeedToMove(double desiredDistanceInches, double distanceFromAprilTagInches) {
        return limelight.hasValidTarget() && (Math.abs((distanceFromAprilTagInches - desiredDistanceInches)) > 0.75d);
    }

    private boolean needToMoveForward(double desiredDistanceInches) {
        return limelight.hasValidTarget() && desiredDistanceInches < limelight.getDistanceFromAprilTagInches();
    }

    private boolean needToMoveBackward(double desiredDistanceInches) {
        return limelight.hasValidTarget() && desiredDistanceInches > limelight.getDistanceFromAprilTagInches();
    }

    public void setDesiredDistanceFromAprilTagInches(double distanceInches) {
        desiredDistanceFromAprilTagInches = distanceInches;
    }
}
