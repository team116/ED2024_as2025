package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

public class MoveArmToAngle extends DurationCommand {
private static final double CLOSE_EPSILON = 5.0;
private static final double EPSILON = 0.1;

    private boolean atTargetAngle;
    private Arm armSubsystem;
    private double targetAngle;
    
    public MoveArmToAngle(Arm arm, double angle, double duration) {
        super(duration);
        armSubsystem = arm;
        this.targetAngle = angle;

        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        atTargetAngle = false;
        SmartDashboard.putBoolean("At Target Angle", atTargetAngle);
    }

    @Override
    public void execute() {
        super.execute();
        double currentAngle = armSubsystem.getAngleDegrees();
        double diff = targetAngle - currentAngle;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            armSubsystem.stop();
            atTargetAngle = true;
        } else {
            if (diff < 0) {
                if (absDiff < CLOSE_EPSILON) {
                    armSubsystem.moveDownSlow();
                } else {
                    armSubsystem.moveDown();
                }
            } else {
                if (absDiff < CLOSE_EPSILON) {
                    armSubsystem.moveUpSlow();
                } else {
                    armSubsystem.moveUp();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        SmartDashboard.putBoolean("At Target Angle", atTargetAngle);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atTargetAngle;
    }
}
