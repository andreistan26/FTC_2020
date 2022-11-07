package RobotParts;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;

import test.GamepadTest.AVXGamepadTest;

import static RobotParts.FasterThanRR.AutoControl;
import static RobotParts.FasterThanRR.moveFieldCentric;
import static RobotParts.FasterThanRR.scaleSpeed;
import static java.lang.Math.max;
import static RobotParts.FasterThanRR.robotTrunSpeed;
import static RobotParts.FasterThanRR.robotXSpeed;
import static RobotParts.FasterThanRR.robotYSpeed;

public class DriveTrain {
    public Motor leftRear, leftFront, rightRear, rightFront;
    private AVXGamepadTest gamepad1, gamepad2;
    private double x, y, t;
    private Telemetry telemetry;
    public double maxVelo = 0;

    public DriveTrain(HardwareMap hM, AVXGamepadTest gamepad1, AVXGamepadTest gamepad2, Telemetry telemetry) {
        ExpansionHubEx eH = hM.get(ExpansionHubEx.class, "Control Hub");

        this.leftRear = new Motor("leftRear", hM, NeveRest20Gearmotor.class, eH).sare().rwe().breakMotor();
        this.leftFront = new Motor("leftFront", hM, NeveRest20Gearmotor.class, eH).sare().rwe().breakMotor();
        this.rightFront = new Motor("rightFront", hM, NeveRest20Gearmotor.class, eH).sare().rwe().breakMotor().reverse();
        this.rightRear = new Motor("rightRear", hM, NeveRest20Gearmotor.class, eH).sare().rwe().breakMotor().reverse();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void update(boolean teleop){
        if(!AutoControl) {
            FasterThanRR.gamepadDriveTrainControl(gamepad1);
        }
        this.x = robotXSpeed;
        this.y = robotYSpeed;
        this.t = robotTrunSpeed;
        double leftFrontPower  = y + t + x;//y + t + x;
        double leftRearPower   = y + t - x;//-y - t + x;
        double rightFrontPower = y - t - x;//-y + t + x;
        double rightRearPower  = y - t + x;//y - t + x;

        if(gamepad1.a.toggle && !AutoControl){
            leftFrontPower*=0.4;
            leftRearPower*=0.4;
            rightRearPower*=0.4;
            rightFrontPower*=0.4;
        }
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void maxVeloDt(){
        if (maxVelo < leftRear.getVelo()) {
            maxVelo = leftRear.getVelo();
        }
        if(maxVelo < leftFront.getVelo()){
            maxVelo = leftFront.getVelo();
        }
        if(maxVelo < rightFront.getVelo()){
            maxVelo = rightFront.getVelo();
        }
        if(maxVelo < rightRear.getVelo()){
            maxVelo = rightRear.getVelo();
        }
        telemetry.addData("leftRear", leftRear.getVelo());
        telemetry.addData("leftFront", leftFront.getVelo());
        telemetry.addData("rightRear", rightRear.getVelo());
        telemetry.addData("rightFront", rightFront.getVelo());
        telemetry.addData("maxVelo", maxVelo);
        telemetry.update();
    }

    public void powerShow(){
        telemetry.addData("leftRear", leftRear.getPower());
        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightRear", rightRear.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.update();
    }

    public void resetEncoders(){
        leftFront.sare().rwe().breakMotor();
        rightRear.sare().rwe().breakMotor();
        leftRear.sare().rwe().breakMotor();
        rightFront.sare().rwe().breakMotor();
    }
}
