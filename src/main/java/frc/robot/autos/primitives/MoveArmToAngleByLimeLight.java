package frc.robot.autos.primitives;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class MoveArmToAngleByLimeLight extends DurationCommand {
    Arm arm;
    Limelight limelight;
    public MoveArmToAngleByLimeLight(Arm armSubsystem, double duration, Limelight limelight) {
        super(duration);
        arm = armSubsystem;
        this.limelight = limelight;

        addRequirements(arm);
    }
}
