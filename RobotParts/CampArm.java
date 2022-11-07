package RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import test.GamepadTest.AVXGamepadTest;

public class CampArm {
    Servo arm;
    public enum State{
        UP, IDLE, MIDDLE, CATCH
    }
    public State state;
    public AVXGamepadTest gamepad;
    public CampArm(HardwareMap hardwareMap, AVXGamepadTest gamepadTest){
        arm = hardwareMap.servo.get("arm");
        state = State.IDLE;
        gamepad = gamepadTest;
    }

    public void update(){
        switch (state){
            case CATCH:
                arm.setPosition(0);
                break;
            case UP:
                arm.setPosition(0.59);
                break;
            case IDLE:
                arm.setPosition(1);
                break;
            case MIDDLE:
                arm.setPosition(0.44);
                break;
        }
        powering();
    }

    public void powering(){
        if(gamepad.right_bumper.value){
            state = State.UP;
        }
        if(gamepad.left_bumper.value){
            state = State.CATCH;
        }




    }
}