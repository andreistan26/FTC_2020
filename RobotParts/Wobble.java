package RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import test.GamepadTest.AVXGamepadTest;

import static RobotParts.FasterThanRR.AutoControl;

public class Wobble {
    private Motor wobble;
    private Servo manuta;
    private AVXGamepadTest gamepad1;
    public static double POWER_UP = 1;
    public static double POWER_DOWN = -1;
    public static double POWER_HOLD = 0;
    public enum State{
        GO_TO_FRONT, GO_TO_SHOOTER, TELEOP, ZERO
    }
    public enum WobbleServoState{
        HOLD, DROP, MIDDLE
    }
    public WobbleServoState servoState;
    public State state;
    public Wobble(HardwareMap hardwareMap, AVXGamepadTest gamepad1){
        this.gamepad1 = gamepad1;
        wobble = new Motor("wobble",hardwareMap,Go_50_9.class).breakMotor().reverse();
        manuta = hardwareMap.servo.get("manuta");
        servoState = WobbleServoState.HOLD;
        state = State.TELEOP;
        powering();
    }

    public void update(){
        if(!AutoControl) {
            wobble.setPower(gamepad1.left_stick_powerY);
            if (gamepad1.y.toggle) {
                servoState = WobbleServoState.DROP;
            } else {
                servoState = WobbleServoState.HOLD;
            }
        }
        powering();
    }

    public void powering(){
        switch (state){
            case GO_TO_FRONT:
                wobble.setPower(-1);
                break;
            case GO_TO_SHOOTER:
                wobble.setPower(1);
                break;
            case ZERO:
                wobble.setPower(0);
                break;
        }

        switch (servoState){
            case DROP:
                manuta.setPosition(0);
                break;
            case HOLD:
                manuta.setPosition(1);
                break;
            case MIDDLE:
                manuta.setPosition(0.31);
                break;
        }
    }
}
