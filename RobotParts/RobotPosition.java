package RobotParts;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utils.MathUtils;

import static RobotParts.ThreeWheelOdometry.*;
import static Utils.MathUtils.angleWrapRad;
import static java.lang.Math.*;

public class RobotPosition {

    public static Pose robotPose = new Pose(0,0,0);
    public static double robotX;
    public static double robotY;
    public static double robotAngle;


    public static void setPosBad(double x, double y, double angle_rad){
        robotPose.x = x;
        robotPose.y = y;
        robotPose.angle = angle_rad;
    }

    public static void resetPosBad(){
        setPosBad(0,0,0);
    }

    public static void resetPos(){
        robotPose.x = 0.0;
        robotPose.y = 0.0;
        robotPose.angle = 0.0;
        ThreeWheelOdometry.reset();
    }

    public static void updatePositionGF(Pose pose, double lastAngle, Telemetry telemetry){
        double angleIncrement = pose.angle;
        double relativeX = pose.x;
        double relativeY = pose.y;

        if(angleIncrement != 0.0){
            double radiusOfMovement = relativeX + angleIncrement;
        }
    }


    public static void updatePosition(Pose pose, double lastAngle, Telemetry telemetry){
        degTraveled += Math.toDegrees(pose.angle);

        double tetha = pose.angle;

        double sinTetha, cosTetha;
        if(abs(tetha) > 1e-6){
            sinTetha = 1 - tetha*tetha/6.0;
            cosTetha = tetha/2;
        }else{
            sinTetha = Math.sin(tetha)/tetha;
            cosTetha = (1.0-Math.cos(tetha))/tetha;
        }

        double move = sinTetha * pose.y - cosTetha * pose.x;
        double strafe = cosTetha * pose.y + sinTetha * pose.x;

        telemetry.addData("move", move);
        telemetry.addData("strafe", strafe);

        Point movePoint = new Point(strafe,move);

        xTraveled += movePoint.x;
        yTraveled += movePoint.y;

        Point finalDelta = MathUtils.pointDelta(movePoint, robotPose.angle);

        robotPose.x += finalDelta.x;
        robotPose.y += finalDelta.y;
        robotPose.angle = angleWrapRad(lastAngle);
    }

    public static void setPosition(double x, double y, double angle){
        robotPose.x = x;
        robotPose.y = y;
        robotPose.angle = angle;

        initRight = currRight;
        initLeft = currLeft;
        lastResetAngle = angle;
    }



}
