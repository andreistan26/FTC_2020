package RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest3_7GearmotorV1;
import com.qualcomm.robotcore.hardware.HardwareMap;

import test.GamepadTest.AVXGamepadTest;

import static RobotParts.FasterThanRR.AutoControl;

@Config
public class IntakeUG {
    private AVXGamepadTest gamepad1;
    private AVXGamepadTest gamepad2;
    public Motor intake;
    public double wobbleTicks;
    private final double POWER_OFF = 0;
    private static double POWER_ON = 1;
    public enum State{
        OFF,INTAKING,SPITTING
    }

    double trigger_kT = 0.2;
    public State state;
    public IntakeUG(HardwareMap hardwareMap,AVXGamepadTest gamepad1, AVXGamepadTest gamepad2){
        this.intake = new Motor("intake", hardwareMap, NeveRest20Gearmotor.class).breakMotor().sare().rwe();
        state = State.OFF;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update(){
        wobbleTicks = intake.getEncoderTicksRaw();
        powering();
        if(!AutoControl) {
            if (gamepad1.right_trigger > trigger_kT || gamepad2.right_trigger > trigger_kT) {
                state = State.INTAKING;
            } else if (gamepad1.left_trigger > trigger_kT || gamepad2.left_trigger > trigger_kT) {
                state = State.SPITTING;
            } else {
                state = State.OFF;
            }
        }
    }
    public void powering(){
        switch (state){
            case OFF: {
                intake.setPower(POWER_OFF);
                break;
            }
            case INTAKING: {
                intake.setPower(POWER_ON);
                break;
            }
            case SPITTING: {
                intake.setPower(-POWER_ON);
                break;
            }
        }
    }
}
