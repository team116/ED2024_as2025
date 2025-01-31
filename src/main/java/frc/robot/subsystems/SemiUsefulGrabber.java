package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SemiUsefulGrabber extends SubsystemBase{
    private SparkMax grabberMotor;
    private DigitalInput grabberLimitSwitch;
    private boolean limitSwitchEnabled = true;

    public SemiUsefulGrabber(){
        grabberMotor = new SparkMax(Constants.GRABBER_MOTOR_ID, MotorType.kBrushless);
        grabberLimitSwitch = new DigitalInput(Constants.GRABBER_LIMIT_SWITCH_CHANNEL);
    }

    public void intakeGamePiece() {
        if (!limitSwitchIsPressed()) {
            grabberMotor.set(0.7);
        } else {
            slowStallIntake();
        }
    }

    public void getRidOfGamePiece() {
        grabberMotor.set(-0.4);
    }

    public void openClaw(){

    }

    public void closeClaw(){
        
    }

    public void setLimitSwitchEnabled(boolean enable) {
        this.limitSwitchEnabled = enable;
    }

    public void disableLimitSwitch() {
        setLimitSwitchEnabled(false);
    }

    public void enableLimitSwitch() {
        setLimitSwitchEnabled(true);
    }

    public boolean limitSwitchIsPressed() {
        return limitSwitchEnabled ? !grabberLimitSwitch.get() : false;
    }

    public void stop() {
        grabberMotor.set(0.0);
    }

    public void slowStallIntake() {
        grabberMotor.set(0.00); // original value 0.04 changed for herndon fest
    }
    
}
