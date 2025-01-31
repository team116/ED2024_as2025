package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroBalancing extends Command{
    private SwerveSubsystem swerve;
    private int isBalancedCount = 0;

    public GyroBalancing(SwerveSubsystem swerveSubsystem){
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double pitch = swerve.getPitch();
        if (pitch == 0.0){
            isBalancedCount++;
        }else if (pitch > 3.0){
            isBalancedCount = 0;
            swerve.setSpeedPercent(0.1);
        }else {
            isBalancedCount = 0;
            swerve.setSpeedPercent(-0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
