package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class SpeakerScoringClose extends SequentialCommandGroup {

    public SpeakerScoringClose(Shooter shooter, IntakeSubsystem intake, Arm arm) {
        ShootAssist shootAssist = new ShootAssist(shooter, intake, arm, Constants.SPEAKER_SHOOTING_ARM_ANGLE, Constants.SPEAKER_SHOOTING_POWER);

        addCommands(shootAssist);
    }
    
}
