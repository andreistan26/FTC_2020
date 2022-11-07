package RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAuto {
    public Servo tablita;
    public enum State{
        DOWN, UP
    }
    public State state;

    public ServoAuto(HardwareMap hardwareMap){
        tablita = hardwareMap.servo.get("tablita");
        state = State.UP;
        update();
    }

    public void update(){
        switch (state){
            case UP:
                tablita.setPosition(0);
                break;
            case DOWN:
                tablita.setPosition(1);
                break;
        }
    }
}
