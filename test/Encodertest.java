package test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RobotParts.BaseOpMode;
@TeleOp
public class Encodertest extends BaseOpMode {

    @Override
    public void onMainLoop() {
        telemetry.addData("shooter1", robot.shooter.shooterMotor1.getEncoderTicksRaw());
        telemetry.addData("shooter1", robot.shooter.shooterMotor2.getEncoderTicksRaw());
        telemetry.addData("leftFront", robot.dt.leftFront.getEncoderTicksRaw());
        telemetry.addData("leftRear", robot.dt.leftRear.getEncoderTicksRaw());
        telemetry.addData("rightFront", robot.dt.rightFront.getEncoderTicksRaw());
        telemetry.addData("rightRear", robot.dt.rightRear.getEncoderTicksRaw());
        telemetry.update();

    }

    @Override
    public void initLoop() {

    }
}
