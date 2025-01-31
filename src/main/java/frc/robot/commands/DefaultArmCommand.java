package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.UselessArm;

public class DefaultArmCommand extends BaseArmCommand {
    private Joystick gunnerLogitech;
    // private Joystick gunnerStation; // FIXME: Don't believe this ended up being necessary

    public DefaultArmCommand(UselessArm armSubSystem, Joystick gunnerLogitech, Joystick gunnerStation) {
        super(armSubSystem);
        this.gunnerLogitech = gunnerLogitech;
        // this.gunnerStation = gunnerStation;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void end(boolean interrupted){
       super.end(interrupted);
    }

    @Override
    protected void checkForDriverInputs() {
        // NOTE: This is airplane mode
        double upDownValue = -gunnerLogitech.getY();
        upDownValue = shape(upDownValue);

        double adjustedUpDownValue = upDownValue > 0 ? upDownValue * 0.3 : upDownValue * 0.4;  // FIXME: Might even go faster than this

        if (Math.abs(adjustedUpDownValue) > 0.03) {
            moveToDesiredPosition = false;
            manualMovementEngaged = true;
            arm.move(adjustedUpDownValue);
        } else if (manualMovementEngaged) {  // Previously was doing manual movement, but no longer, so turn it off
            manualMovementEngaged = false;
            arm.stop();
            desiredCanCoderPosition = arm.getCANCoderPosition();
        }
    }

    @Override
    protected void checkForMoveToPositionRequests() {

        UselessArm.Position desiredArmPosition = null;

        if (gunnerLogitech.getRawButtonPressed(1)) {
            desiredArmPosition = UselessArm.Position.STOWED;
        } else if (gunnerLogitech.getRawButtonPressed(2)) {
            desiredArmPosition = UselessArm.Position.HUMAN_PLAYER_STATION;
        } else if (gunnerLogitech.getRawButtonPressed(7)) {
            desiredArmPosition = UselessArm.Position.CONE_HIGH_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(8)) {
            desiredArmPosition = UselessArm.Position.CUBE_HIGH_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(9)) {
            desiredArmPosition = UselessArm.Position.CONE_MID_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(10)) {
            desiredArmPosition = UselessArm.Position.CUBE_MID_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(11)) {
            desiredArmPosition = UselessArm.Position.LOW_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(12)) {
            desiredArmPosition = UselessArm.Position.FLOOR_INTAKE;
        }

        // Raw 1 - Trigger - Stowed in robot ()
        // Raw 2 - Human Intake

        // y-axis -> "up"/"down"   (invert, as negative)

        // Raw 7  - Cone High
        // Raw 8  - Cube High
        // Raw 9  - Cone Mid
        // Raw 10 - Cube Mid
        // Raw 11 - Low Score
        // Raw 12 - Floor Intake

        if (desiredArmPosition != null) {
            desiredCanCoderPosition = desiredArmPosition.getAngleDegrees();
            //System.out.println("Requested desired position: " + desiredCanCoderPosition);
            moveToDesiredPosition = true;
        }
    }

    public static double shape(double start) {
        if (start < 0){
          return -(start * start);
        }
        return start * start;
    }
}
