package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UselessLeds;

public class DefaultLedCommand extends Command {
    private UselessLeds leds;
    private Joystick gunnerLogitech;
    private Joystick gunnerStation;
    private Alliance alliance;

    public DefaultLedCommand(UselessLeds leds, Joystick gunnerLogitech, Joystick gunnerStation) {
        this.leds = leds;
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerStation = gunnerStation;
        addRequirements(this.leds);
    }

    @Override
    public void initialize() {
        // alliance = DriverStation.getAlliance();
    }

    @Override
    public void execute() {
        if (gunnerLogitech.getRawButton(6)) {  // FIXME: Might want as gunner station switch to "hold"
            setLedsToDesiredElementColor();
        } else {
            setLedsToAllianceColor();
        }
    }

    private void setLedsToDesiredElementColor() {
        if (gunnerStation.getRawButton(7)) {  // FIXME: Button 3, 5, or something else?  [1,2,4] already in use
            leds.setColor(UselessLeds.Color.YELLOW);
        } else {
            leds.setColor(UselessLeds.Color.PURPLE);
        }
    }

    private void setLedsToAllianceColor() {
        if (alliance == DriverStation.Alliance.Red) {
            leds.setColor(UselessLeds.Color.RED);
        } else {
            leds.setColor(UselessLeds.Color.BLUE);
        }
    }
}
