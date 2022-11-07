package RobotParts;


import com.qualcomm.robotcore.util.Range;

import Utils.Gamepad.AVXGamepad;
import test.GamepadTest.AVXGamepadTest;

import static RobotParts.RobotPosition.robotPose;
import static Utils.MathUtils.angleWrapRad;
import static java.lang.Math.max;

public class FasterThanRR {
    public static double robotXSpeed;
    public static double robotYSpeed;
    public static double robotTrunSpeed;

    public static boolean AutoControl = false;

    public static void gamepadDriveTrainControl(AVXGamepadTest gamepad){
        robotXSpeed = gamepad.left_stick_powerX;
        robotYSpeed = gamepad.left_stick_powerY;
        robotTrunSpeed = gamepad.right_stick_x;

        if(Math.abs(robotYSpeed)<0.05){
            robotYSpeed = 0.0;
        }

        if(Math.abs(robotXSpeed)<0.05){
            robotXSpeed = 0.0;
        }

        if(Math.abs(robotTrunSpeed)<0.05){
            robotTrunSpeed = 0.0;
        }
    }

    public static void scaleSpeed(double a, double b, double c, double d, double value){
        a *=value;  b*=value;   c*=value;   d*=value;
    }

    public static void scaleSpeed(){
        double max = max(robotYSpeed, robotXSpeed);
        robotXSpeed /=max;
        robotYSpeed /=max;
    }


    public static void moveFieldCentric(double x, double y, double turn){
        double hypot = Math.hypot(x,y);
        double direction = angleWrapRad(Math.atan2(x,y)-robotPose.angle);
        double sin = Math.sin(direction);
        double cos = Math.cos(direction);
        robotXSpeed = sin * hypot;
        robotYSpeed = cos * hypot;
        robotTrunSpeed = turn;

    }

    public static void reset(){
        robotTrunSpeed = 0;
        robotXSpeed = 0;
        robotYSpeed = 0;
    }
    public static boolean isRobotStopped(){
        return robotTrunSpeed == 0 && robotXSpeed == 0 && robotYSpeed == 0;
    }

    public static void clipAllSpeeds(double maxSpeed){
        robotYSpeed = Range.clip(robotYSpeed, -maxSpeed, maxSpeed);
        robotXSpeed = Range.clip(robotXSpeed, -maxSpeed, maxSpeed);
        robotTrunSpeed = Range.clip(robotTrunSpeed, -maxSpeed, maxSpeed);
    }

    public static void clipSpeeds(double maxSpeed){
        robotYSpeed = Range.clip(robotYSpeed, -maxSpeed, maxSpeed);
        robotXSpeed = Range.clip(robotXSpeed, -maxSpeed, maxSpeed);
    }

    public static void clipSpeeds(double maxSpeed, double ... speeds){
        for(double speed : speeds){
            speed = Range.clip(speed, -maxSpeed, maxSpeed);
        }
    }

}
