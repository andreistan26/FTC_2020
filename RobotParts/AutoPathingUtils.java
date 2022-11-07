package RobotParts;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoPathingUtils {
    private static final double ROBOT_DISTANCE_KT = 0.5;

    public static boolean robotDistToPoint(Point targetPoint, double dist)
    {
        return Math.hypot(targetPoint.x - RobotPosition.robotPose.x, targetPoint.y - RobotPosition.robotPose.y) < dist;
    }

    public static boolean robotDistToY(double y, double dist){
        return y-RobotPosition.robotPose.y < dist;
    }
    public static boolean robotDistToPoint(Point targetPoint)
    {
        return Math.hypot(targetPoint.x - RobotPosition.robotPose.x, targetPoint.y - RobotPosition.robotPose.y) < 1;
    }




}
