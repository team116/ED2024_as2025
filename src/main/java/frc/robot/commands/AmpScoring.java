package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class AmpScoring extends SequentialCommandGroup {

    public AmpScoring(Shooter shooter, IntakeSubsystem intake, Arm arm) {
        ShootAssist shootAssist = new ShootAssist(shooter, intake, arm, Constants.AMP_SCORING_ARM_ANGLE, Constants.AMP_SCORING_POWER);

        addCommands(shootAssist);
    }
    
}
