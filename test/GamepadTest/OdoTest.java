package test.GamepadTest;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import RobotParts.BaseOpMode;
import RobotParts.ButtonAction;
import RobotParts.CampArm;
import RobotParts.FasterThanRR;
import RobotParts.Flicker;
import RobotParts.Odometry;
import RobotParts.PidDriveAuto;
import RobotParts.Pose;
import RobotParts.Robot;
import RobotParts.ServoAuto;
import RobotParts.Shooter;
import RobotParts.Wobble;
import test.AutoBlue;

import static RobotParts.RobotPosition.resetPos;
import static RobotParts.RobotPosition.robotAngle;
import static RobotParts.RobotPosition.robotPose;
import static RobotParts.ThreeWheelOdometry.resetPosTo;
import static RobotParts.FasterThanRR.*;
import static java.lang.Math.abs;

@Config
@TeleOp
public class OdoTest extends BaseOpMode  {
    enum State{
        PS_FIRST,PS_SECOND,PS_THIRD, DONE
    }
    enum WobblState{
        GO_PICK_UP, OVER_FENCE,DRIVER_CONTROLLED
    }
    double PICK_UP_TICKS = 3800;
    double OVER_FENCE = 2700;
    private boolean[] shot = {false, false, false};
    long time = 0, lastTime = 0;
    int iter, liter = 0;
    public static double TRACK_WIDTH = 32.59;
    public static double AUX_TRACK_WIDTH =3;
    public static double T_T=1, T_Y=1, T_X=1;
    PidDriveAuto pidDrive;
    public static double P_X = 0.025, I_X = 0, D_X = 0.009, F_X = 0.1;
    public static double P_Y = 0.025, I_Y = 0, D_Y = 0.009, F_Y = 0;
    public static double P_T = 1.2 ,I_T = 0, D_T = 0.3, F_T = 0.1;
    boolean reseted = false;
    State state;
    WobblState wstate;
    @Override
    public void initLoop() {
        wstate = WobblState.DRIVER_CONTROLLED;
        state = State.PS_FIRST;
        AutoControl = false;
        Robot.resetAll();
        resetPos();
        pidDrive = new PidDriveAuto();
        time = System.currentTimeMillis();
        lastTime = time;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.arm.state = CampArm.State.UP;
        pidDrive.xController.setThreshold(0.5);
        pidDrive.yController.setThreshold(0.5);
        pidDrive.tethaController.setThreshold(Math.toRadians(0.4));
    }

    @Override
    public void onMainLoop() {
        robot.updateTeleop();
        telemetry.addData("state",state);

        if(avxGamepad2.dpad_down.value){
            wstate = WobblState.GO_PICK_UP;
        }
        if(avxGamepad2.dpad_up.value){
            wstate = WobblState.OVER_FENCE;
        }
        if(abs(avxGamepad2.left_stick_y) > 0.1){
            wstate = WobblState.DRIVER_CONTROLLED;
        }

        switch (wstate){
            case GO_PICK_UP:
                if(abs(robot.intakeUG.wobbleTicks) < PICK_UP_TICKS){
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                break;
            case OVER_FENCE:
                if(abs(robot.intakeUG.wobbleTicks) > OVER_FENCE){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                break;
            case DRIVER_CONTROLLED:
                robot.wobble.state = Wobble.State.TELEOP;
                break;
        }
           if(avxGamepad2.a.toggle){
               Shooter.TARGET_VELO_BASE = 1700;
               Flicker.DELAY = 500;
                AutoControl = true;
                switch (state) {
                    case PS_FIRST: {
                        pidDrive.tethaController.setThreshold(Math.toRadians(0.1));//0.5
                        if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                            robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                        if (pidDrive.goToPositionFC(-102, -18, -5.5,0.5,0.5,0.15 )) {
                            if (!shot[0]) {
                                if (robot.buttonAction.flick_auto()) {
                                    Log.v("AUTO_FirstSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                    shot[0] = true;
                                    break;
                                }
                            } else {
                                if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                                    state = State.PS_SECOND;
                                }
                            }
                        }
                        break;
                    }
                    case PS_SECOND: {
                        if(robot.servoAuto.state == ServoAuto.State.DOWN)
                            robot.servoAuto.state = ServoAuto.State.UP;

                        if (pidDrive.goToPositionFC(-102, -18, 1.7,0.6,0.5,0.5,0.15 )) {
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
                        }
                        break;
                    }
                    case PS_THIRD: {
                        if (pidDrive.goToPositionFC(-102, -18, 8.4, 0.6,0.5,0.5,0.15 )) {
                            if (!shot[2]) {
                                if (robot.buttonAction.flick_auto()) {
                                    Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                    shot[2] = true;
                                    break;
                                }
                            } else {
                                if (robot.buttonAction.state== ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                                    state = State.DONE;
                                }
                            }
                        }
                        break;
                    }
                    case DONE:
                        AutoControl = false;
                        break;
                }
           }else{
                Shooter.TARGET_VELO_BASE = 1930;
                Flicker.DELAY = 150;
                AutoControl = false;
           }

           telemetry.addData("tickswo", robot.intakeUG.wobbleTicks);

        if(avxGamepad2.x.value){
            reseted = true;
            resetPosTo(new Pose(0.0,0.0,0.0));
        telemetry.update();
    }
}
