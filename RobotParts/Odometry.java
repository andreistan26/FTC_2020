package RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import kotlin.text.CharDirectionality;

import static RobotParts.RobotPosition.robotPose;

@Config
public class Odometry {
    //cm
   public  static double TRACK_WIDTH = 32.52;
    public  static double AUX_TRACK_WIDTH = 3.0;
   public final static double WHEEL_DIAMETER = 3.8;
   public final static double CPR = 1440.0;
   public final static double Y_TRACK_WIDTH = 0.0;
   public final static double X_MULTIPLIER = 0.989913;
   public final static double Y_MULTIPLIER = 0.99590;
   private DriveTrain dt;
   private Imu imu;

    public static double leftEncoderTicks = 0;
    public static double rightEncoderTicks = 0;
    public static double auxEncoderTicks = 0;

    public static double cmPerTicks = WHEEL_DIAMETER*Math.PI/CPR;


    public Odometry(HardwareMap hMap, DriveTrain dt){
        this.dt = dt;
    }

    public void initOdometry(){

    }

    public void update(Telemetry telemetry){
        auxEncoderTicks = dt.leftRear.getEncoderTicksRaw();
        rightEncoderTicks = dt.leftFront.getEncoderTicksRaw();
        leftEncoderTicks = dt.rightFront.getEncoderTicksRaw();
        telemetry.addData("leftFront  right| initial read", rightEncoderTicks);
        telemetry.addData("rightFront left | initial read", leftEncoderTicks);
        telemetry.addData("leftRear aux | initial read", auxEncoderTicks);
        telemetry.addData("X pos", robotPose.x);
        telemetry.addData("Y pos", robotPose.y);
        telemetry.addData("Angle", Math.toDegrees(robotPose.angle));
        telemetry.addData("TRACKWIDTH", TRACK_WIDTH);


        ThreeWheelOdometry.updateBad(leftEncoderTicks, rightEncoderTicks, auxEncoderTicks, telemetry);


    }


}