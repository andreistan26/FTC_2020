package RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.SocketException;
import java.util.List;
import java.util.concurrent.TimeUnit;

import test.GamepadTest.AVXGamepadTest;

import static RobotParts.ThreeWheelOdometry.initReset;
@Config
public abstract class BaseOpMode extends LinearOpMode {

    public Odometry odometry;
    public AVXGamepadTest avxGamepad, avxGamepad2;
    public Robot robot ;
    public ElapsedTime timeSinceStart;
    public void initMode(){
        avxGamepad = new AVXGamepadTest(gamepad1, telemetry);
        avxGamepad2 = new AVXGamepadTest(gamepad2, telemetry);
        robot = new Robot(hardwareMap,telemetry,avxGamepad,avxGamepad2);
        odometry = new Odometry(hardwareMap, robot.dt);
        RobotPosition.resetPosBad();
        initReset();
        initLoop();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initMode();
        waitForStart();
//        robot.socket.start();
//        timeSinceStart.reset();
        while(opModeIsActive()){
            for(LynxModule hub : robot.allHubs)
            {
                hub.clearBulkCache();
            }
            odometry.update(telemetry);
            onMainLoop();
            if(isStopRequested()) return;
        }
    }



    public abstract void onMainLoop();
    public abstract void initLoop();
}
