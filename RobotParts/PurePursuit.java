package RobotParts;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static RobotParts.RobotPosition.robotAngle;
import static RobotParts.RobotPosition.robotPose;
import static RobotParts.RobotPosition.robotX;
import static RobotParts.RobotPosition.robotY;
import static Utils.MathUtils.angleWrapRad;
import static fastaf.LaDoiPasi.stopDrive;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.min;
import static java.lang.Math.abs;


import java.util.ArrayList;

import fastaf.LaDoiPasi;

public class PurePursuit {

    public static boolean finishingMove = false;
    public static boolean followCurve(ArrayList<CurvePoint> allPoints, PidDriveAuto pidAuto, double followAngle_deg, double clip){
        //TODO add last point in array opmode
        int location = whereAmI(allPoints);
        double followDistance = allPoints.get(location).followDistance;
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robotPose.x, robotPose.y),followDistance);
        CurvePoint last = allPoints.get(allPoints.size()-2);
        if(whereAmI(allPoints) == allPoints.size()-2 || hypot(last.x - robotPose.x, last.y - robotPose.y) < 20.0) {
            boolean fin = pidAuto.goToPositionFC(last.x, last.y, last.tetha_deg, clip, true);

            if(fin){
                stopDrive(true);
                pidAuto.resetAll();
                return true;
            }
        }else{
            LaDoiPasi.goToPositionProp(followMe.x, followMe.y, followAngle_deg, clip);
        }
//        telemetry.addData("point index", location);
//        telemetry.addData("point x ", allPoints.get(location).x);
//        telemetry.addData("point y ", allPoints.get(location).y);
        return false;
    }

    private static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
        if(Math.abs(linePoint1.y - linePoint2.y)< 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y-linePoint1.y)/(linePoint2.x - linePoint1.x);

        double quadradicA = 1.0 + pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadradicB = (2.0 * m1 * y1) - (2.0* pow(m1,2)*x1);

        double quadradicC = ((pow(m1,2) * pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1,2) - pow(radius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadradicB + sqrt(pow(quadradicB,2)-(4.0 * quadradicA *quadradicC))) /(2.0 * quadradicA);
            double yRoot1 = m1 * (xRoot1 -x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = min(linePoint1.x, linePoint2.x);
            double maxX = max(linePoint1.x,linePoint2.x);

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadradicB - sqrt(pow(quadradicB,2)-(4.0 * quadradicA *quadradicC))) /(2.0 * quadradicA);
            double yRoot2 = m1 * (xRoot2 -x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        }catch (Exception e){

        }
        return allPoints;
    }

    private static int whereAmI(ArrayList<CurvePoint> allPoints){
        int index = 0;
        double closestDistance = 10000000;

        for(int i=1;i<allPoints.size()-1;i++){
            CurvePoint lastCurvePoint = allPoints.get(i-1);
            CurvePoint currentCurvePoint = allPoints.get(i);
            CurvePoint futureCurvePoint = allPoints.get(i+1);


            double d1 = linePointProjection(lastCurvePoint.toPoint(),currentCurvePoint.toPoint());
            double d2 = linePointProjection(currentCurvePoint.toPoint(),futureCurvePoint.toPoint());


            if(closestDistance > d1){
                closestDistance = d1;
                index = i; 
            }
            if(closestDistance>d2){
                closestDistance = d2;
                index = i+1;
            }
        }

        return index;
    }

    private static double linePointProjection(Point p1, Point p2){
        double distance = 9999999;
        double minX = min(p1.x ,p2.x);
        double maxX = max(p1.x ,p2.x);

        double minY = min(p1.y ,p2.y);
        double maxY = max(p1.y ,p2.y);

        if(abs(p1.x - p2.x) < 0.03){
            if(robotPose.y<maxY && robotPose.y > minY){
                return abs(robotPose.x - minX);
            }
        }

        if(abs(p1.y - p2.y) < 0.03){
            if(robotPose.x<maxX && robotPose.x > minX){
                return abs(robotPose.y - minY);
            }
        }
        double m1 = (p2.y-p1.y)/(p2.x-p1.x);
        double b1 = p2.y - m1*p2.x;
        double m2 = -(1/m1);
        double b2 = robotPose.y - (m2 * robotPose.x);
        double x = (m1 * b2 - m1 * b1) / (m1 * m1 + 1);
        double y = m2*x + b2;
        if(x < maxX && x > minX){
            distance = hypot(x-robotPose.x  ,y-robotPose.y);
        }
        return distance;
    }

    private static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));// goes to the first point in the path if it doesn'nt have any intersections

        for(int i=0;i<pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation,followRadius,startLine.toPoint(),
                    endLine.toPoint());

            double closestAngle = 1000000;

            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.x - robotPose.x,thisIntersection.y - robotPose.y);
                double deltaAngle = Math.abs(angleWrapRad(angle - robotPose.angle));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void addLastPoint(ArrayList<CurvePoint> allPoints){
        CurvePoint lastPoint = new CurvePoint(allPoints.get(allPoints.size()-1));
        double distance = lastPoint.followDistance ;
        CurvePoint lastCurvePoint = allPoints.get(allPoints.size()-1);
        Point p1 = allPoints.get(allPoints.size()-2).toPoint();
        Point p2 = allPoints.get(allPoints.size()-1).toPoint();

        allPoints.add(new CurvePoint(p2.x + (p2.x - p1.x)/hypot(p2.x-p1.x,p2.y-p2.x)*distance, p2.y + (p2.y - p1.y)/hypot(p2.x-p1.x,p2.y-p2.x)*distance , lastCurvePoint.moveSpeed, lastCurvePoint.turnSpeed,
                0, lastCurvePoint.slowDownTurnRadians, lastCurvePoint.slowDownTurnAmount));
    }

    public static void changeLastPoint(ArrayList<CurvePoint> allPoints, CurvePoint curvePoint){
        allPoints.remove(allPoints.size()-1);
        allPoints.remove(allPoints.size()-1);
        allPoints.add(curvePoint);
        addLastPoint(allPoints);
    }
}