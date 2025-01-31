package frc.robot.autos.primitives;

import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeMotorAnyDirection extends DurationCommand {
    private IntakeSubsystem intake;
    private double speed = 1.0;
    // private double distance; // Might not need to use this

    public enum RollerDirection {
        CONSUME,
        EXPEL
    }

    private final RollerDirection direction;

    public RunIntakeMotorAnyDirection(IntakeSubsystem intake, double maxTimeout, RollerDirection direction) {
        super(maxTimeout);
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (direction == RollerDirection.CONSUME) {
            intake.runRollersToConsume();
        } else {
            intake.runRollersToVomit();
        } 
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotor();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
