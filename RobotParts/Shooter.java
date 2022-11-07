package RobotParts;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import test.GamepadTest.AVXGamepadTest;
//2100 0.006 0 0.0001 0.2
//1900 0.0052
public class Shooter {
    public Motor shooterMotor1;
    public Motor shooterMotor2;
    private double shooterPower;
    public PIDFController pid;
    private AVXGamepadTest gamepad;
    private Telemetry telemetry;
    public static double TARGET_VELO_BASE = 1930;
    public double speed;
    public double velo;

    public enum ShooterState{
        OFF, GETTING_TO_SPEED, AT_SPEED
    }

    public ShooterState shooterState;
    public Shooter(HardwareMap hardwareMap, AVXGamepadTest gamepad, Telemetry telemetry){
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.shooterMotor1 = new Motor("shooter1", hardwareMap, NeveRest40Gearmotor.class).breakMotor().rwe().sare().rwe().reverse();
        this.shooterMotor2 = new Motor("shooter2", hardwareMap, NeveRest40Gearmotor.class).breakMotor().rwe().sare().rwe().reverse();
        this.telemetry = telemetry;
        shooterState = ShooterState.OFF;
        pid = new PIDFController(new double[]{0.0063,0.0014,0,0.2});

    }

    public void test(){
        velo = shooterMotor1.getVelo();
        speed = 1.0;
        if(gamepad.b.value){
            stop();
        }
    }

    public void update(){
        velo = shooterMotor1.getVelo(); 
        switch (shooterState) {
            case OFF :
//                velo = 0;
                stop();
                pid.reset();
                break;
            case GETTING_TO_SPEED:
                    power();
                    break;
                }else{
                    shooterState = ShooterState.AT_SPEED;
                    break;
                }
            case AT_SPEED:
                power();
                if(!checkPower())
                     shooterState = ShooterState.GETTING_TO_SPEED;
                break;
        }
    }

    public void start(){
        shooterState = ShooterState.GETTING_TO_SPEED;
    }

    public void power(){
        double power = pid.calculate(velo, TARGET_VELO_BASE, System.currentTimeMillis());
        if(TARGET_VELO_BASE > 2000){
            power = Range.clip(power, 0.73, 1);

        }else{
            power = Range.clip(power, 0.59, 1);
        }
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public boolean checkPower(){
        return Math.abs(velo- TARGET_VELO_BASE) < 150;
    }

    public void setPid(double p, double i, double d, double f){
        pid.changeVars(new double[]{p, i, d, f});
    }

    public void setTarget(double target){
        TARGET_VELO_BASE = target;
    }

    public void stop(){
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

}
