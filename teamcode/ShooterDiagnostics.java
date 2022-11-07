package teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RobotParts.BaseOpMode;
import RobotParts.ButtonAction;
import RobotParts.Flicker;
import RobotParts.ServoAuto;
import RobotParts.Shooter;
import test.GamepadTest.OdoTest;

import static RobotParts.Robot.resetAll;
import static RobotParts.RobotPosition.robotPose;

@TeleOp
@Config
public class ShooterDiagnostics extends BaseOpMode{
    enum State{
        PS_FIRST,PS_SECOND,PS_THIRD, DONE
    }

    public static double velo_target = 2100;
    public static double p=0.0063,i=0.0009,d=0,f=0.2;
    public static double delay=90, position = 0.5, start = 0.58 ;
    State state;
    boolean auto_aim;
    boolean []shot = {false, false, false};
    @Override
    public void onMainLoop() {
        robot.updateTeleop();
        if(avxGamepad.x.value){
            robot.shooter.setPid(p,i,d,f);
        }

        if(avxGamepad.b.value){
            robot.shooter.setTarget(velo_target);
            Flicker.DELAY = delay;
            Flicker.TO_SHOOT_POSITION = position;
            Flicker.IDLE_POSITION = start;
        }

        if(avxGamepad.y.value){
            auto_aim = true;
            state = State.PS_FIRST;
        }

        if(auto_aim) {
            switch (state) {
                case PS_FIRST: {
                    if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                        robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                    if (!shot[0]) {
                        if(robot.shooter.velo > Shooter.TARGET_VELO_BASE)
                        if (robot.buttonAction.flick_auto()) {
                            shot[0] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = State.PS_SECOND;
                        }
                    }
                    break;
                }
                case PS_SECOND: {
                    if (robot.servoAuto.state == ServoAuto.State.DOWN)
                        robot.servoAuto.state = ServoAuto.State.UP;
                    if (!shot[1]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_SecondSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[1] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = State.PS_THIRD;
                        }
                    }
                    break;
                }
                case PS_THIRD: {
                    if (!shot[2]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[2] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = State.DONE;
                        }
                    }
                    break;
                }
                case DONE: {
                    robot.shooter.shooterState = Shooter.ShooterState.OFF;
                    shot[0] = false;
                    shot[1] = false;
                    shot[2] = false;
                    auto_aim = false;
                }
            }
        }

        telemetry.addData("target", velo_target);
        telemetry.addData("motor velo", robot.shooter.velo);
        telemetry.addData("gamepad pressed right button", (Math.abs(avxGamepad.right_bumper.pressed-avxGamepad.left_bumper.pressed)%11.0)/10.0);
        telemetry.addData("shooter1", robot.shooter.shooterMotor1.getPower());
        telemetry.addData("shooter2", robot.shooter.shooterMotor2.getPower());
        telemetry.addData("shooterstate", robot.shooter.shooterState);
        telemetry.addData("flickerPos", robot.flicker.statePosition);
        telemetry.addData("flickerReady", robot.flicker.stateTimeReady);
        telemetry.addData("rightBumper VALUE:", avxGamepad.right_bumper.value);
        telemetry.addData("wobble ticks", robot.intakeUG.wobbleTicks);
        telemetry.update();
    }

    @Override
    public void initLoop() {
        state = State.DONE;
        auto_aim = false;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        dashboard.setTelemetryTransmissionInterval(25);
        resetAll();
    }
}
