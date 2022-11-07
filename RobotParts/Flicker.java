package RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import test.GamepadTest.AVXGamepadTest;
//@Config
public class Flicker {
    public Servo flicker;
    AVXGamepadTest gamepad;
    long currentTime, lastTime;
    public enum StatePosition {
        IDLE, TO_SHOOT;
    }

    public enum StateTimeReady{
        NOT_READY, READY
    }
    private boolean manualControl;

    public StatePosition statePosition;
    public StateTimeReady stateTimeReady;

    public static double IDLE_POSITION = 0.58;
    public static double TO_SHOOT_POSITION = 0.45;
    public static double DELAY = 170;//230; //120;

    public boolean flickerReturning = false;

    public Flicker(HardwareMap hardwareMap, AVXGamepadTest gamepad){
        flicker = hardwareMap.servo.get("flicker");
        this.gamepad = gamepad;
        statePosition = StatePosition.IDLE;
        stateTimeReady = StateTimeReady.READY;
        currentTime = System.currentTimeMillis();
        lastTime = currentTime;
        manualControl = false;
        flicker.setPosition(IDLE_POSITION);
        flickerReturning = false;
    }

    public void manual() {
        if (gamepad.y.toggle) {
            flicker.setPosition(TO_SHOOT_POSITION);
        } else {
                flicker.setPosition(IDLE_POSITION);
        }
    }
    public void shoot(){
        statePosition = StatePosition.TO_SHOOT;
        update();
    }

    public void update() {
        currentTime = System.currentTimeMillis();
        switch (statePosition) {
            case IDLE:
                flicker.setPosition(IDLE_POSITION);
                if (stateTimeReady == StateTimeReady.NOT_READY || flickerReturning) {
                    if (timeProfile(DELAY /2)) {
                        stateTimeReady = StateTimeReady.READY;
                        flickerReturning = false;
                    }
                }else{
                    lastTime = System.currentTimeMillis();
                }
                break;
            case TO_SHOOT:
                flicker.setPosition(TO_SHOOT_POSITION);
                stateTimeReady = StateTimeReady.NOT_READY;
                if (timeProfile(DELAY / 2)) {
                    statePosition = StatePosition.IDLE;
                    flickerReturning = true;
                    lastTime = System.currentTimeMillis();
                }
                break;

        }
    }



    public boolean timeProfile(double time){
        if(currentTime - lastTime > time){
            return true;
        }
        return false;
    }
}
