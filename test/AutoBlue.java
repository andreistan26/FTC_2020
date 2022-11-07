package test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;

import java.util.ArrayList;

import RobotParts.AutoPathingUtils;
import RobotParts.BaseOpMode;
import RobotParts.ButtonAction;
import RobotParts.CampArm;
import RobotParts.CurvePoint;
import RobotParts.FasterThanRR;
import RobotParts.Flicker;
import RobotParts.IntakeUG;
import RobotParts.PIDFController;
import RobotParts.PidDriveAuto;
import RobotParts.Point;
import RobotParts.PurePursuit;
import RobotParts.Robot;
import RobotParts.ServoAuto;
import RobotParts.Shooter;
import RobotParts.Wobble;
import Utils.UGRectDetector;

import static RobotParts.FasterThanRR.isRobotStopped;
import static RobotParts.FasterThanRR.reset;
import static RobotParts.RobotPosition.resetPos;
import static RobotParts.RobotPosition.robotPose;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
@Config
@Autonomous
public class AutoBlue extends BaseOpMode {
    boolean f;
    int caz;
    enum PATHING_STATES{
        PS_FIRST, PS_SECOND, PS_THIRD, FIRST_WOBBLE, GO_RING_STACK, SHOOT_RING_STACK, GRAB_SECOND_WOBBLE, PUT_SECOND_WOBBLE, PARK, SHOOT_RING_STACK_2, GO_RING_STACK_2,GO_BACK, GO_BOUNCE_BACK, SHOOT_BOUNCE
    }
    private PidDriveAuto pidDrive;
    private boolean[] shot = {false, false, false, false, false, false, false, false, false, false, false, false, false};
    private PATHING_STATES state;
    private ArrayList<CurvePoint> path_ps_1;
    private ArrayList<CurvePoint> path_ps_for_zero;

    private boolean TURN_PARK = false;
    double angleFor4;
    int i;
    @Override
    public void onMainLoop() {
        robot.updateAuto();
        i++;
        switch (caz) {
            case 0:
                zeroNou();
                break;
            case 1:
                unuNou();
                break;
            case 2:
                patru();
                break;
        }
    }

    @Override
    public void initLoop() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Robot.resetAll();
//        Shooter.TARGET_VELO_BASE = 1800;
        Shooter.TARGET_VELO_BASE = 1750;
        FasterThanRR.AutoControl = true;
        f = false;
        shot = new boolean[]{false, false, false, false, false, false, false, false, false, false, false, false, false};
        state = PATHING_STATES.PS_FIRST;
        pidDrive = new PidDriveAuto();
        path_ps_1 = new ArrayList<CurvePoint>();
        path_ps_1.add(new CurvePoint(34,-335, -135, 5));
        path_ps_1.add(new CurvePoint(-80.5,-301, -90, 5));

        path_ps_for_zero = new ArrayList<CurvePoint>();
        path_ps_for_zero.add(new CurvePoint(0, -290,-135, 10));
        path_ps_for_zero.add(new CurvePoint(-75,-301, -135, 10));
        PurePursuit.addLastPoint(path_ps_1);
        PurePursuit.addLastPoint(path_ps_for_zero);

        Flicker.DELAY = 220;
        pidDrive.tethaController.setThreshold(Math.toRadians(0.4));//0.5
        pidDrive.xController.setThreshold(0.3);
        pidDrive.yController.setThreshold(0.3);

        UGRectDetector detector = new UGRectDetector(hardwareMap, "Webcam 1");
        detector.init();
        FtcDashboard.getInstance().startCameraStream(detector.camera, 30);

        while(!isStopRequested() && !isStarted()){
            caz = detector.getStack().ordinal();
            telemetry.addData("caz ", caz);
            telemetry.update();
        }

    }
    void zeroNou(){
        switch (state){
            case PS_FIRST: {
                robot.arm.state = CampArm.State.CATCH;
                pidDrive.tethaController.setThreshold(Math.toRadians(0.15));//0.5
                if(robot.servoAuto.state == ServoAuto.State.UP)
                    robot.servoAuto.state = ServoAuto.State.DOWN;
                if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                if (pidDrive.goToPositionFC(0, -160, -0.65,0.65,0.5,0.5,0.2 )) { // era -0.8 cand megea bine
                    if (!shot[0]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_FirstSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[0] = true;
                            robot.servoAuto.state = ServoAuto.State.UP;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_SECOND;
                        }
                    }
                }
                break;
            }
            case PS_SECOND: {
                robot.arm.state = CampArm.State.UP;
                if(robot.servoAuto.state == ServoAuto.State.DOWN)
                    robot.servoAuto.state = ServoAuto.State.UP;

                if (pidDrive.goToPositionFC(0, -160, 6,0.6,0.5,0.5,0.23 )) {
                    if (!shot[1]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_SecondSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[1] = true;
//                            Flicker.DELAY=400;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_THIRD;
                        }
                    }
                }
                break;
            }
            case PS_THIRD: {
                if (pidDrive.goToPositionFC(0, -160, 11.30, 0.6,0.5,0.5,0.23 )) {
                    if (!shot[2]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[2] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.FIRST_WOBBLE;
                            Flicker.DELAY = 190;
//                            Flicker.DELAY = 160;
                            pidDrive.tethaController.setThreshold(Math.toRadians(0.4));//0.5
                            pidDrive.xController.setThreshold(0.5);
                            pidDrive.yController.setThreshold(0.5);

                        }
                    }
                }
                break;
            }

            case FIRST_WOBBLE: {
                if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                    robot.shooter.shooterState = Shooter.ShooterState.OFF;
                if (pidDrive.goToPositionFC(70, -196, -90)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    state = PATHING_STATES.GRAB_SECOND_WOBBLE;
                }
                break;
            }
            case GRAB_SECOND_WOBBLE: {
                if (abs(robot.intakeUG.wobbleTicks) < 3796 - 30)
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                else
                    robot.wobble.state = Wobble.State.ZERO;
                if(AutoPathingUtils.robotDistToY(-85, 1.5))
                    robot.wobble.servoState = Wobble.WobbleServoState.HOLD;
                if (pidDrive.goToPositionFC(63,-83, 0, 0.69)) {
                    state = PATHING_STATES.PUT_SECOND_WOBBLE;
                    sleep(700);
                    reset();
                }
                break;
            }
            case PUT_SECOND_WOBBLE: {
                if(abs(robot.intakeUG.wobbleTicks)>3500){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                if(AutoPathingUtils.robotDistToPoint(new Point( 50,-143), 4.4)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }
                if (pidDrive.goToPositionFC(58, -147.5, 135,1,1,1)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    state = PATHING_STATES.GO_BACK;
                }
                break;
            }

            case GO_BACK:{
                robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                Shooter.TARGET_VELO_BASE = 2050;
                if(pidDrive.goToPositionFC(48,-138, -135,2,2,2)){
                    state = PATHING_STATES.GO_RING_STACK_2;
                }
                break;
            }

            case GO_RING_STACK_2:{
                if(abs(robot.intakeUG.wobbleTicks)>200){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                if(AutoPathingUtils.robotDistToY(-290, 7)){
                    state = PATHING_STATES.GO_BOUNCE_BACK;
                }
                pidDrive.goToPositionFC(0, -290, -135, 3, 3, 3);
            }

            case GO_BOUNCE_BACK:{
                if(abs(robot.intakeUG.wobbleTicks)>200){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if(PurePursuit.followCurve(path_ps_for_zero, pidDrive, -135, 0.8)){
                    state = PATHING_STATES.SHOOT_BOUNCE;
                }
                break;
            }

            case SHOOT_BOUNCE:{
                if(robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if (pidDrive.goToPositionFC(32, -142, -2)) {
                    Log.v("AUTO_First", "DONE");
                    if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                        if (!shot[10]) {
                            if (robot.buttonAction.flick_auto()) {
                                Log.v("AUTO_FirstSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                shot[10] = true;
                            } else {
                                break;
                            }
                        }else{
                            state = PATHING_STATES.PARK;
                        }
                }
                break;
            }

            case PARK:
                pidDrive.tethaController.setThreshold(Math.toRadians(0.4));//0.5
                if(pidDrive.goToPositionFC(0,-180,135)){
                    stop();
                }
        }
    }

    void unuNou(){
        switch (state) {
            case PS_FIRST: {
                robot.arm.state = CampArm.State.CATCH;
                pidDrive.tethaController.setThreshold(Math.toRadians(0.15));//0.5
                if(robot.servoAuto.state == ServoAuto.State.UP)
                    robot.servoAuto.state = ServoAuto.State.DOWN;
                if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                if (pidDrive.goToPositionFC(0, -160, -0.65,0.65,0.5,0.5,0.2 )) { // era -0.8 cand megea bine
                    if (!shot[0]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_FirstSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[0] = true;
                            robot.servoAuto.state = ServoAuto.State.UP;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_SECOND;
                        }
                    }
                }
                break;
            }
            case PS_SECOND: {
                robot.arm.state = CampArm.State.UP;
                if(robot.servoAuto.state == ServoAuto.State.DOWN)
                    robot.servoAuto.state = ServoAuto.State.UP;

                if (pidDrive.goToPositionFC(0, -160, 6,0.6,0.5,0.5,0.23 )) {
                    if (!shot[1]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_SecondSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[1] = true;
//                            Flicker.DELAY=400;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_THIRD;
                        }
                    }
                }
                break;
            }
            case PS_THIRD: {
                if (pidDrive.goToPositionFC(0, -160, 11.30, 0.6,0.5,0.5,0.23 )) {
                    if (!shot[2]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[2] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.FIRST_WOBBLE;
                            Flicker.DELAY = 190;
//                            Flicker.DELAY = 160;
                            pidDrive.tethaController.setThreshold(Math.toRadians(0.4));//0.5
                            pidDrive.xController.setThreshold(0.5);
                            pidDrive.yController.setThreshold(0.5);

                        }
                    }
                }
                break;
            }

            case FIRST_WOBBLE: {
                if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                    robot.shooter.shooterState = Shooter.ShooterState.OFF;
                if (pidDrive.goToPositionFC(30, -232, 0)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.MIDDLE;
                    state = PATHING_STATES.GO_RING_STACK;
                }
                break;
            }

            case GO_RING_STACK:{
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if(pidDrive.goToPositionFC(30, -90, 0)){
                    state = PATHING_STATES.SHOOT_RING_STACK;
                    Shooter.TARGET_VELO_BASE = 2050;
                }
                break;
            }

            case SHOOT_RING_STACK:{
                if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                if (pidDrive.goToPositionFC(32, -148, 0)) {
                    Log.v("AUTO_First", "DONE");
                    if (!shot[3]) {
                        if (robot.buttonAction.flick_auto()) {
                            shot[3] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            robot.intakeUG.state = IntakeUG.State.OFF;
                            state = PATHING_STATES.GRAB_SECOND_WOBBLE;
                            robot.shooter.shooterState = Shooter.ShooterState.OFF;
                            robot.wobble.state = Wobble.State.GO_TO_FRONT;
                        }
                    }
                }
                break;
            }

            case GRAB_SECOND_WOBBLE: {

                if (abs(robot.intakeUG.wobbleTicks) < 3797 - 50)
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                else
                    robot.wobble.state = Wobble.State.ZERO;
                if(AutoPathingUtils.robotDistToY(-83, 1.5))
                    robot.wobble.servoState = Wobble.WobbleServoState.HOLD;
                if (pidDrive.goToPositionFC(63,-81, 0, 0.45)) {
                    state = PATHING_STATES.PUT_SECOND_WOBBLE;
                    reset();
                    sleep(400);
                }
                break;
            }
            case PUT_SECOND_WOBBLE: {
                if(abs(robot.intakeUG.wobbleTicks)>3300){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                if(AutoPathingUtils.robotDistToPoint(new Point( -33,-237), 2.4)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }
                if (pidDrive.goToPositionFC(-33, -237, 90)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    state = PATHING_STATES.PARK;
                }
                break;
            }
            case PARK:{
                if(robot.intakeUG.wobbleTicks < 50){
                    robot.wobble.state= Wobble.State.ZERO;
                }
                if(pidDrive.goToPositionFC(-20, -175, 180)){
                    reset();
                    stop();
                }
                break;
            }


        }
    }

    void patru(){
        switch(state) {
            case PS_FIRST: {
                robot.arm.state = CampArm.State.CATCH;
                pidDrive.tethaController.setThreshold(Math.toRadians(0.15));//0.5
                if(robot.servoAuto.state == ServoAuto.State.UP)
                    robot.servoAuto.state = ServoAuto.State.DOWN;
                if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                if (pidDrive.goToPositionFC(0, -160, -0.65,0.65,0.5,0.5,0.2 )) { // era -0.8 cand megea bine
                    if (!shot[0]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_FirstSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[0] = true;
                            robot.servoAuto.state = ServoAuto.State.UP;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_SECOND;
                        }
                    }
                }
                break;
            }
            case PS_SECOND: {
                robot.arm.state = CampArm.State.UP;
                if(robot.servoAuto.state == ServoAuto.State.DOWN)
                    robot.servoAuto.state = ServoAuto.State.UP;

                if (pidDrive.goToPositionFC(0, -160, 6,0.6,0.5,0.5,0.23 )) {
                    if (!shot[1]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_SecondSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[1] = true;
//                            Flicker.DELAY=400;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.PS_THIRD;
                        }
                    }
                }
                break;
            }
            case PS_THIRD: {
                if (pidDrive.goToPositionFC(0, -160, 11.30, 0.6,0.5,0.5,0.23 )) {
                    if (!shot[2]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[2] = true;
                            break;
                        }
                    } else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            state = PATHING_STATES.FIRST_WOBBLE;
                            Flicker.DELAY = 190;
//                            Flicker.DELAY = 160;
                            pidDrive.tethaController.setThreshold(Math.toRadians(0.4));//0.5
                            pidDrive.xController.setThreshold(0.5);
                            pidDrive.yController.setThreshold(0.5);

                        }
                    }
                }
                break;
            }

            case FIRST_WOBBLE: {
                if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                    robot.shooter.shooterState = Shooter.ShooterState.OFF;
                if (pidDrive.goToPositionFC(86, -286, 0,4,4,1.2)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.MIDDLE;
                    state = PATHING_STATES.GO_RING_STACK;
                }
                break;
            }

            case GO_RING_STACK:{
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if(robotPose.y > -236){
                    if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                        robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                    if(pidDrive.goToPositionFC(32,-147, angleFor4, 0.12)){
                        state = PATHING_STATES.SHOOT_RING_STACK;
                        Shooter.TARGET_VELO_BASE = 2080;
                        Flicker.DELAY = 140;
                        angleFor4 = Math.toDegrees(robotPose.angle);
                        Log.v("AUTO_FirstRingStack", "DONE" + " angleFor4: " + angleFor4);
                    }
                }else {
                    pidDrive.goToPositionFC(32, -210, 0.6);
                    angleFor4 = Math.toDegrees(robotPose.angle);
                }
                break;
            }

            case SHOOT_RING_STACK: {
                if (robot.shooter.shooterState != Shooter.ShooterState.AT_SPEED)
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                if (pidDrive.goToPositionFC(32, -142, -2)) {
                    Log.v("AUTO_First", "DONE");
                    if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                    if (!shot[3]) {
                        if (robot.buttonAction.flick_auto()) {
                            Log.v("AUTO_FirstSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                            shot[3] = true;
                        }else{
                            break;
                        }
                    }else {
                        if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                            if (!shot[4]) {
                                if (robot.buttonAction.flick_auto()) {
                                    Log.v("AUTO_SecondSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                    shot[4] = true;

//                                    Flicker.DELAY=150;
                                }else{
                                    break;
                                }
                            } else {
                                if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                                    if (!shot[5]) {
                                        if (robot.buttonAction.flick_auto()) {
                                            Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                            shot[5] = true;
                                        }else{
                                            break;
                                        }
                                    }else{
                                        state = PATHING_STATES.GO_RING_STACK_2;
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            }

            case GO_RING_STACK_2:{
                Log.v("RingStack2", "RUNNING" + " angle: " + Math.toDegrees(robotPose.angle));
                robot.wobble.state = Wobble.State.GO_TO_FRONT;
                if(pidDrive.goToPositionFC( 32 ,-109, angleFor4+1)){
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                    state = PATHING_STATES.GRAB_SECOND_WOBBLE;
                    robot.shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
                    Flicker.DELAY = 240;
                    angleFor4 = Math.toDegrees(robotPose.angle);
                }
                break;
            }

            case SHOOT_RING_STACK_2: {
                if (abs(robot.intakeUG.wobbleTicks) > 3797 - 150)
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                else
                    robot.wobble.state = Wobble.State.ZERO;
                if (pidDrive.goToPositionFC(32, -145, angleFor4-4)) {
                    if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                        if (!shot[6]) {
                            if (robot.buttonAction.flick_auto()) {
                                Log.v("AUTO_FirstSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                shot[6] = true;
                            }else{
                                break;
                            }
                        }else {
                            if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                                if (!shot[7]) {
                                    if (robot.buttonAction.flick_auto()) {
                                        Log.v("AUTO_SecondSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                        shot[7] = true;
                                    }else{
                                        break;
                                    }
                                } else {
                                    if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT) {
                                        if (!shot[8]) {
                                            if (robot.buttonAction.flick_auto()) {
                                                Log.v("AUTO_ThirdSHOT", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                                shot[8] = true;
                                            }else{
                                                break;
                                            }
                                        }else{
                                            state = PATHING_STATES.PUT_SECOND_WOBBLE;
                                        }
                                    }
                                }
                            }
                        }
                }
                break;
            }


            case GRAB_SECOND_WOBBLE: {

                Log.v("AUTO_GoGrabWobble2", "RUNNING" + " angle: " + Math.toDegrees(robotPose.angle));
                if (abs(robot.intakeUG.wobbleTicks) < 3797 - 20)
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                else
                    robot.wobble.state = Wobble.State.ZERO;
                if(AutoPathingUtils.robotDistToY(-80.5, 0.4))
                    robot.wobble.servoState = Wobble.WobbleServoState.HOLD;
                if(pidDrive.goToPositionFC(75,-79.5, angleFor4)) {
                    Log.v("AUTO_GoGrabWobble2", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                    state = PATHING_STATES.SHOOT_RING_STACK_2;
                    angleFor4 = Math.toDegrees(robotPose.angle);
                    pidDrive.xController.setThreshold(0.7);
                    break;
                }
                break;
            }

            case PUT_SECOND_WOBBLE: {
                if(abs(robot.intakeUG.wobbleTicks)>3300){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                if (pidDrive.goToPositionFC(60, -265, angleFor4+137.7, 2, 2, 2)) {
                    robot.wobble.servoState = Wobble.WobbleServoState.DROP;
                    state = PATHING_STATES.GO_BACK;
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }
                break;
            }

            case GO_BACK:{
                robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                if(pidDrive.goToPositionFC(30,-265, angleFor4+137.7,5,5,5)){
                    state = PATHING_STATES.GO_BOUNCE_BACK;
                }
                break;
            }

            case GO_BOUNCE_BACK:{
                if(abs(robot.intakeUG.wobbleTicks)>200){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if(PurePursuit.followCurve(path_ps_1, pidDrive, -135, 1)){
                    state = PATHING_STATES.SHOOT_BOUNCE;
                }
                break;
            }

            case SHOOT_BOUNCE:{
                if(abs(robot.intakeUG.wobbleTicks)>1000){
                    robot.wobble.state = Wobble.State.GO_TO_FRONT;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                robot.intakeUG.state = IntakeUG.State.INTAKING;
                if (pidDrive.goToPositionFC(32, -142, -2)) {
                    Log.v("AUTO_First", "DONE");
                    if (robot.buttonAction.state == ButtonAction.ButtonActionState.READY_TO_SHOOT)
                        if (!shot[10]) {
                            if (robot.buttonAction.flick_auto()) {
                                Log.v("AUTO_FirstSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                                shot[10] = true;
                            } else {
                                break;
                            }
                        }else{
                            state = PATHING_STATES.PARK;
                        }
                }
                break;
            }

            case PARK:{
                if(abs(robot.intakeUG.wobbleTicks)>100){
                    robot.wobble.state = Wobble.State.GO_TO_SHOOTER;
                }else{
                    robot.wobble.state = Wobble.State.ZERO;
                }
                if(pidDrive.goToPositionFC(13, -175, 0, 5, 5, 5)){
                    reset();
                    stop();
                }
                break;
            }
        }
    }

}
