package RobotParts;


import Utils.MathUtils;

import static RobotParts.RobotPosition.robotPose;

public class Speedometer {
    private static double lastUpdateTime = 0.0;

    public static double xCmTraveled;
    public static double yCmTraveled;

    private static double lastAngle = 0;
    private static double angularVel = 0;

    public static Point robotSpeed;

    public double getDegPerSec(){
        return Math.toDegrees(angularVel);
    }

    public static void update(){
        double time = System.currentTimeMillis()/1000.0;
        double dt = time - lastUpdateTime;

        double xSpeed = xCmTraveled / dt;
        double ySpeed = yCmTraveled / dt;

        angularVel = (robotPose.angle - lastAngle) / dt;
        lastAngle = robotPose.angle;

//        xCmTraveled = 0.0;
//        yCmTraveled = 0.0;

        robotSpeed = new Point(xSpeed, ySpeed);
        robotSpeed = MathUtils.pointDelta(robotSpeed, robotPose.angle);
        lastUpdateTime = time;

    }
}
