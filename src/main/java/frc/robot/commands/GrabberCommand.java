package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SemiUsefulGrabber;

public class GrabberCommand extends Command {

    private SemiUsefulGrabber grabber;

    public GrabberCommand(SemiUsefulGrabber grabberSubSystem) {
        this.grabber = grabberSubSystem;
        addRequirements(grabberSubSystem);
    }

    @Override
    public void initialize() {
        // System.out.println("Begin Default Grabber Command");
        grabber.stop();
    }

    @Override
    public void execute() {
        grabber.slowStallIntake();

        /*
        if (grabber.limitSwitchIsPressed()) {
            grabber.slowStallIntake();
        } else {
            grabber.stop();
        }
        */

    }

    @Override
    public void end(boolean interrupted){
        // System.out.println("End Default Grabber Command");
        grabber.stop();
    }
}
