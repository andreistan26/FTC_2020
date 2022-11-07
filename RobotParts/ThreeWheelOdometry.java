package RobotParts;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import static RobotParts.Odometry.*;
import static RobotParts.RobotPosition.robotPose;
import static Utils.MathUtils.angleWrapRad;

public class ThreeWheelOdometry {

    public static double degTraveled = 0.0;
    public static double xTraveled = 0.0;
    public static double yTraveled = 0.0;

    private static double last_leftEncoder = 0;
    private static double last_rightEncoder = 0;
    private static double last_auxEncoder = 0;
    private static double last_imuAngle = 0;

    public static double initRight = 0.0;
    public static double initLeft = 0.0;
    public static double initAux = 0.0;

    public static double currLeft = 0.0;
    public static double currRight = 0.0;
    public static double currAux = 0.0;

    private static double last_angle = 0.0;
    public static double lastResetAngle = 0.0;

    public static void updateHybrid(double leftEncoder, double rightEncoder, double auxEncoder){

        currLeft = leftEncoder;
        currRight = rightEncoder;
        currAux = auxEncoder;

        double dt_leftEncoder = (leftEncoder - last_leftEncoder - initLeft)*cmPerTicks;
        double dt_rightEncoder = (rightEncoder - last_rightEncoder - initRight)*cmPerTicks;
        double dt_auxEncoder = (auxEncoder - last_auxEncoder - initAux)*cmPerTicks;

        double angle = (dt_leftEncoder-dt_rightEncoder) / TRACK_WIDTH - lastResetAngle;

        double leftTotal = (leftEncoder - initLeft)*cmPerTicks;
        double rightTotal = (rightEncoder - initRight)*cmPerTicks;
        last_angle = robotPose.angle;
        robotPose.angle = angleWrapRad((leftTotal-rightTotal)/TRACK_WIDTH);

        double auxPrediction = angle * AUX_TRACK_WIDTH;

        double yDelta = (dt_leftEncoder+dt_rightEncoder) / 2.0;
        double xDelta = dt_auxEncoder - auxPrediction;

        if(Math.abs(angle) > 0) {
            double radiusOfMovement = (dt_rightEncoder + dt_leftEncoder) / (2 * angle);
            double radiusOfStraif = xDelta/angle;


            yDelta = (radiusOfMovement * Math.sin(angle)) - (radiusOfStraif * (1-Math.cos(angle)));
            xDelta = radiusOfMovement * (1-Math.cos(angle)) + (radiusOfStraif * Math.sin(angle));

        }

        robotPose.y += (Math.cos(last_angle)*yDelta) + (Math.sin(last_angle) * xDelta);
        robotPose.x += (Math.sin(last_angle)*yDelta) - (Math.cos(last_angle) * xDelta);

        last_auxEncoder = auxEncoder;
        last_rightEncoder = rightEncoder;
        last_leftEncoder = leftEncoder;
    }
    public static void resetPosTo(Pose finalPose){
        robotPose.resetValues(finalPose.x, finalPose.y, finalPose.angle);
        initRight += currRight;
        initAux += currAux;
        initLeft += currLeft;
        reset();
    }



    public static void update(double leftEncoder, double rightEncoder, double auxEncoder){
        currLeft = -leftEncoder;
        currRight = rightEncoder;
        currAux = auxEncoder;

        double wheelLeftDelta = currLeft - last_leftEncoder; //20
        double wheelRightDelta = currRight - last_rightEncoder;
        double wheelAuxDelta = currAux - last_auxEncoder;

        double leftScaled = wheelLeftDelta*cmPerTicks/1000.0;//200,
        double rightScaled = wheelRightDelta*cmPerTicks/1000.0;
        double auxScaled = wheelAuxDelta*cmPerTicks/1000.0;

        double angleIncrement = (wheelLeftDelta-wheelRightDelta)*TRACK_WIDTH/100000.0; // 20

        double rightTotal = currRight - initRight;
        double leftTotal = -(currLeft - initLeft);

        double world_last_angle = robotPose.angle;
        robotPose.angle = angleWrapRad((leftTotal-rightTotal)*TRACK_WIDTH/100000.0 + lastResetAngle);

        double trackerPrediciton = Math.toDegrees(angleIncrement)*(AUX_TRACK_WIDTH/10);

        double r_xDistance = auxScaled - trackerPrediciton;

        double relativeY = (leftScaled+rightScaled)/2;
        double relativeX = r_xDistance;

        if(Math.abs(angleIncrement) > 0){
            double radiusOfMovement = (rightScaled + leftScaled) / (2 * angleIncrement);
            double radiusOfStraif = r_xDistance/angleIncrement;


            relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1-Math.cos(angleIncrement)));
            relativeX = radiusOfMovement * (1-Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

        }

        robotPose.x += (Math.cos(world_last_angle)*relativeY) + (Math.sin(world_last_angle) * relativeX);
        robotPose.y += (Math.sin(world_last_angle)*relativeY) - (Math.cos(world_last_angle) * relativeX);

        last_auxEncoder = currAux;
        last_rightEncoder = currRight;
        last_leftEncoder = currLeft;
    }


    public static void setPosition(double x, double y, double angle){
        robotPose.x = x;
        robotPose.y = y;
        robotPose.angle = angle;

        initRight = currRight;
        initLeft = currLeft;
        initAux = currAux;

        last_auxEncoder = 0;
        last_rightEncoder = 0;
        last_leftEncoder = 0;
    }

    public static void updateBad(double leftEncoder, double rightEncoder, double auxEncoder, Telemetry telemetry){
        currRight = rightEncoder-initRight;
        currLeft = leftEncoder-initLeft;
        currAux = auxEncoder-initAux;
        double dt_leftEncoder = (currLeft - last_leftEncoder)*cmPerTicks;
        double dt_rightEncoder = (currRight - last_rightEncoder)*cmPerTicks;
        double dt_auxEncoder = (currAux - last_auxEncoder)*cmPerTicks;

        double leftTotal = currLeft*cmPerTicks;
        double rightTotal = currRight*cmPerTicks;
        telemetry.addData("LeftTotal ", leftTotal);
        telemetry.addData("RightTotal", rightTotal);
        telemetry.addData("initRight", initRight);
        telemetry.addData("initLeft", initLeft);
        telemetry.addData("initAux", initAux);

        // RETARD METHOD
//
//        double angle = (dt_leftEncoder - dt_rightEncoder);
//        if(angle > 0) {
//            angle /= TRACK_WIDTH;
//        }else{
//            angle /= TRACK_WIDTH_CCW;
//        }
//        last_angle += angle;
//        last_angle = angleWrapRad(last_angle);

        // NORMAL METHOD


        double angle = (dt_leftEncoder-dt_rightEncoder)/TRACK_WIDTH;
        last_angle = angleWrapRad((leftTotal-rightTotal)/TRACK_WIDTH);
//        telemetry.addData("angle", angle);
//        telemetry.addData("last_angle", last_angle);

        double auxPrediction = angle * AUX_TRACK_WIDTH;

        double yDelta = (dt_leftEncoder+dt_rightEncoder)/ 2.0;
        double xDelta = dt_auxEncoder - auxPrediction;



        KtPoseTest.INSTANCE.updatePosPoseExp(new Pose(xDelta,yDelta,angle), last_angle);

        last_leftEncoder = currLeft;
        last_auxEncoder = currAux;
        last_rightEncoder = currRight;

    }

    public static void updateImuPoseExp(double rightEncoder, double auxEncoder, double imuAngle){
        currRight = rightEncoder;//-initRight;
        currAux = auxEncoder;//-initAux;
        double dt_rightEncoder = (currRight - last_rightEncoder)*cmPerTicks;
        double dt_auxEncoder = (currAux - last_auxEncoder)*cmPerTicks;

        double angle = imuAngle - last_imuAngle;
        last_angle = angleWrapRad(imuAngle);

        double auxPrediction = angle * AUX_TRACK_WIDTH;
        double rightPrediction = angle * TRACK_WIDTH/2;
        double yDelta = dt_rightEncoder - rightPrediction;
        double xDelta = dt_auxEncoder - auxPrediction;



        KtPoseTest.INSTANCE.updatePosPoseExp(new Pose(xDelta,yDelta,angle), last_angle);

        last_imuAngle = imuAngle;
        last_auxEncoder = currAux;
        last_rightEncoder = currRight;
    }

    public static void initReset(){
        initLeft = 0;
        initRight = 0;
        initAux = 0;
    }

    public static void reset(){
        last_angle = 0.0;
        last_leftEncoder = 0.0;
        last_rightEncoder = 0.0;
        last_auxEncoder = 0.0;
    }
}
