package RobotParts;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.net.SocketException;
import java.util.List;

import Utils.Gamepad.AVXGamepad;
import test.GamepadTest.AVXGamepadTest;

import static RobotParts.FasterThanRR.reset;
import static RobotParts.RobotPosition.resetPos;
public class Robot {
    AVXGamepadTest gamepad1;
    AVXGamepadTest gamepad2;
    public DriveTrain dt;
    public IntakeUG intakeUG;
    private Telemetry telemetry;
    public Imu imu;
    public Shooter shooter;
    public Flicker flicker;
    public ButtonAction buttonAction;
    public Wobble wobble;
    public ServoAuto servoAuto;
    public CampArm arm;
    public List<LynxModule> allHubs;
    private VoltageSensor voltageSensor;
    public static double VOLTAGE = 1;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AVXGamepadTest gamepad1, AVXGamepadTest gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        dt = new DriveTrain(hardwareMap, gamepad1, gamepad2, telemetry);
        intakeUG = new IntakeUG(hardwareMap, gamepad1, gamepad2);
        shooter = new Shooter(hardwareMap, gamepad1, telemetry);
        flicker = new Flicker(hardwareMap, gamepad1);
        buttonAction = new ButtonAction(gamepad1, flicker, shooter);
        wobble = new Wobble(hardwareMap, gamepad2 );
        servoAuto = new ServoAuto(hardwareMap);
        arm = new CampArm(hardwareMap, gamepad2);
        setHubsCaching(hardwareMap);
    }
    public void socket() throws SocketException {
//        socket = new UDPSocket();
    }

    public void updateTest(){
        dt.update(true);
        shooter.test();
        flicker.manual(); 
        intakeUG.update();
        wobble.update();
    }

    public void updateTeleop(){
        dt.update(true);
        intakeUG.update();
        shooter.update();
        flicker.update();
        buttonAction.update();
        wobble.update();
        gamepad1.run();
        gamepad2.run();
        servoAuto.update();
        arm.update();
    }

    public void updateAuto(){
        dt.update(false);
        intakeUG.update();
        shooter.update();
        flicker.update();
        buttonAction.update();
        wobble.update();
        servoAuto.update();
        arm.update();
    }

    public void resetDrive(){
        dt.resetEncoders();
    }

    public static void resetAll(){
        resetPos();
        reset();
        ThreeWheelOdometry.currAux = 0;
        ThreeWheelOdometry.currLeft = 0;
        ThreeWheelOdometry.currRight = 0;
    }

    public void maxVelo(){
        dt.maxVeloDt();
        dt.powerShow();
    }

    public void setHubsCaching(HardwareMap hardwareMap)
    {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs)
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
}
