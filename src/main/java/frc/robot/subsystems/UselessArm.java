package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings(value = { "removal" })
public class UselessArm extends SubsystemBase{
    private SparkMax armMotor;
    private CANCoder armCanCoder;
    private SparkLimitSwitch armTopLimitSwitch;
    private SparkLimitSwitch armBottomLimitSwitch;
    private SparkClosedLoopController armMotorController;
    private RelativeEncoder armEncoder;
    private double desiredCANCoderPosition;

    public enum Position{
        CONE_HIGH_GOAL(35.068359375), // 32.255859375
        CONE_MID_GOAL(20.3), // 17.490234375
        CUBE_HIGH_GOAL(18.017578125),
        CUBE_MID_GOAL(-1.845703125),
        LOW_GOAL(-37.373046875),
        HUMAN_PLAYER_STATION(13.623046875),
        FLOOR_INTAKE(-77.431640625),
        STOWED(-93.427734375),
        AUTO_CONE_HIGH_GOAL(38.0),
        AUTO_CONE_HIGH_GOAL_SCORE(24.0),
        AUTO_CONE_MID_GOAL_SCORE(9.0);

        private final double angleDegrees;

        Position(double angleDegrees){
            this.angleDegrees = angleDegrees;
        }

        public double getAngleDegrees(){
            return angleDegrees;
        }
    }

    public UselessArm(){
        // FIXME: Totally irrelevant class, should remove anyway
        /*
        armMotor = new SparkMax(Constants.USELESS_ARM_MOTOR_ID, MotorType.kBrushless);
        // armMotor.setIdleMode(IdleMode.kBrake);

        armCanCoder = new CANCoder(Constants.USELESS_ARM_CAN_CODER_ID);
        armCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0.0);

        armTopLimitSwitch = armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        armTopLimitSwitch.enableLimitSwitch(true);
        armBottomLimitSwitch = armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        armBottomLimitSwitch.enableLimitSwitch(false);
        
        armMotorController = armMotor.getClosedLoopController();
        armMotorController.setP(30.0);// original value before playing with is 0.001, 30 looks like it works nicely
        armMotorController.setOutputRange(-0.2, 0.2);
        armMotorController.setFF(0.0);
        armMotor.burnFlash();
        */
    }

    public void moveUp(){
        armMotor.set(-0.3);
    }

    public void stop(){
        armMotor.set(0);
    }

    public void moveDown(){
        armMotor.set(0.2);
    }

    /**
     * Up is negative...
     */
    public void move(double percentagePower) {
        armMotor.set(percentagePower);
    }

    public void nudgeUp(double positivePercentage) {
        armMotor.set(-positivePercentage);
    }

    public void nudgeDown(double positivePercentage) {
        armMotor.set(positivePercentage);
    }

    public void holdWithPower(double percentage) {
        armMotor.set(-percentage);
    }

    public void moveToPos(UselessArm.Position desiredArmPosition) {
        this.setDesiredCANCoderPosition(desiredArmPosition.angleDegrees);
    }

    public void enableLimitSwitches(){
        //armTopLimitSwitch.enableLimitSwitch(true);
    }

    public void disableLimitSwitches(){
        //armBottomLimitSwitch.enableLimitSwitch(false);
       // armTopLimitSwitch.enableLimitSwitch(false);
    }

    public double getEncoder(){
        return armMotor.getEncoder().getPosition();
    }

    public void resetArmEncoder(){
        armEncoder.setPosition(0.0);
    }

    public void setDesiredCANCoderPosition(double desiredPosition) {
        this.desiredCANCoderPosition = desiredPosition;
    }

    public void setDesiredCANCoderPositionToCurrentPosition() {
        this.desiredCANCoderPosition = getCANCoderPosition();
    }

    public double getDesiredCANCoderPosition() {
        return this.desiredCANCoderPosition;
    }

    public double getCANCoderPosition() {
        return armCanCoder.getAbsolutePosition();
    }
}
