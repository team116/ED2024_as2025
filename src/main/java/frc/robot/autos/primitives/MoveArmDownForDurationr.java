package frc.robot.autos.primitives;

import frc.robot.subsystems.Arm;

public class MoveArmDownForDurationr extends DurationCommand {

    private Arm armSubsystem;

    public MoveArmDownForDurationr(Arm arm, double duration) {
        super(duration);
        armSubsystem = arm;

        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        armSubsystem.moveDown();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
