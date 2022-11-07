package fastaf;

import com.qualcomm.robotcore.util.Range;


import RobotParts.Point;
import RobotParts.Pose;
import RobotParts.Speedometer;
import Utils.Gamepad.AVXGamepad;
import Utils.MathUtils;

import static RobotParts.FasterThanRR.*;
import static RobotParts.RobotPosition.*;
import static Utils.MathUtils.angleWrapRad;
import static java.lang.Math.max;

public class LaDoiPasi {

    //TURN
    private static double turnKp = 0.035;
    private static double turnKd = 0.003;
    //MOVE
    private static double moveKp = 0.15;
    private static double moveKd = 0.03;

    private static double lastErrorX = 0.0;
    private static double lastErrorY = 0.0;
    private static double lastErrorTurn = 0.0;

    public static void goToPosition(double x, double y, double speed, double preferredAngle, double turnSpeed, boolean finalMove) {

        double distanceToTarget = Math.hypot(x - robotPose.x, y - robotPose.y);

        double absoluteAngleToHeading = Math.atan2(y - robotPose.y, x - robotPose.x);

        double relativeAngleToPoint = angleWrapRad(absoluteAngleToHeading - (robotPose.angle - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        if(finalMove){


        }else{
            robotXSpeed = movementXPower * speed;
            robotYSpeed = movementYPower * speed;
            scaleSpeed();
            double relavtiveTurnAngle = relativeAngleToPoint - Math.toRadians(180) + Math.toRadians(preferredAngle);
            robotTrunSpeed = Range.clip(relavtiveTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        }

    }




    public static void goToPositionProp(double x, double y, double followAngle_deg){
        double deltaX = x-robotPose.x;
        double deltaY = y-robotPose.y;
        double angle = Math.atan2(deltaX,deltaY)+Math.toRadians(followAngle_deg);
        double deltaT = angleWrapRad(angle-robotPose.angle);

        double absSpeed = Math.abs(deltaX)+Math.abs(deltaY);
        double m_x = deltaX/absSpeed;
        double m_y = deltaY/absSpeed;
        double m_t = (deltaT)*0.4;
        double max = max(m_x,m_y);
        Range.clip(m_t,-1,1);
        moveFieldCentric(m_x,m_y,m_t);

    }

    public static void goToPositionProp(double x, double y, double followAngle_deg, double clip){
        double deltaX = x-robotPose.x;
        double deltaY = y-robotPose.y;
        double angle = Math.atan2(deltaX,deltaY)+Math.toRadians(followAngle_deg);
        double deltaT = angleWrapRad(angle-robotPose.angle);

        double absSpeed = Math.abs(deltaX)+Math.abs(deltaY);
        double m_x = deltaX/absSpeed;
        double m_y = deltaY/absSpeed;
        double m_t = (deltaT)*0.4;
        double max = max(m_x,m_y);
        Range.clip(m_t,-1,1);
        moveFieldCentric(m_x,m_y,m_t);
        clipSpeeds(clip);

    }


    public void gamepadDriveTrainControl(AVXGamepad gamepad){
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

    public static void stopDrive(boolean stopDrive, boolean stopTurn){
        if(stopDrive){
            robotXSpeed = 0;
            robotYSpeed = 0;
        }
        robotTrunSpeed = stopTurn ? 0 : robotTrunSpeed;
    }

    public static void stopDrive(boolean stopDrive){
        stopDrive(stopDrive, stopDrive);
    }
}
