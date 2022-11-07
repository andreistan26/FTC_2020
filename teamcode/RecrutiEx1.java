package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleopdf")
public class RecrutiEx1 extends OpMode{

    DcMotor leftRear, leftFront, rightRear, rightFront;
    Servo servo;
    @Override
    public void init() {
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo = hardwareMap.servo.get("servoNume");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0);
    }

    @Override
    public void loop() {
        leftRear.setPower(gamepad1.left_stick_y);
        leftFront.setPower(gamepad1.left_stick_y);
        rightFront.setPower(gamepad1.left_stick_y);
        rightRear.setPower(gamepad1.left_stick_y);
        telemetry.addData("leftRear ticks ", leftRear.getCurrentPosition());
        telemetry.update();
        if(gamepad1.a){
            servo.setPosition(1);
        }else{
            servo.setPosition(0);
        }
    }
}
