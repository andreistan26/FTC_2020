package RobotParts;

import android.util.Log;

import RobotParts.PIDFController;

import static RobotParts.FasterThanRR.clipAllSpeeds;
import static RobotParts.FasterThanRR.clipSpeeds;
import static RobotParts.FasterThanRR.reset;
import static RobotParts.FasterThanRR.robotTrunSpeed;
import static RobotParts.FasterThanRR.robotXSpeed;
import static RobotParts.FasterThanRR.robotYSpeed;
import static Utils.MathUtils.angleWrapRad;
import static fastaf.LaDoiPasi.stopDrive;
import static java.lang.Math.*;
import static RobotParts.RobotPosition.*;

public class PidDriveAuto {
    public PIDFController xController;
    public PIDFController yController;
    public PIDFController tethaController;
    public boolean finishedD = false, finishedT = false;
    private static final String TAG = "PidDriveAuto";
    public PidDriveAuto() {
        xController = new PIDFController(new double[]{0.009, 0.0, 0.00123, 0.2},0,0,0.5);
        yController = new PIDFController(new double[]{0.009, 0.0, 0.00123, 0.15},0,0,0.5);
        tethaController = new PIDFController(new double[]{1.1, 0.0, 0.1, 0.1},0,0,0.03);
    }

    public boolean goToPositionPID(double x, double y, double tetha_deg){
        double deltaX = x-robotPose.x;
        double deltaY = y-robotPose.y;
        double distanceToTarget = hypot(deltaX, deltaY);
        double headingAbsolute = atan2(deltaY, deltaX);
        double headingToPoint = angleWrapRad(headingAbsolute - (robotPose.angle)); //- toRadians(90)));
        double relativeX = cos(headingToPoint) * distanceToTarget;
        double relativeY = sin(headingToPoint) * distanceToTarget;


        double m_x = xController.calculate(0, relativeX, System.currentTimeMillis());
        double m_y = yController.calculate(0, relativeY, System.currentTimeMillis());
        double deltaT = angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());


        robotXSpeed = m_x;
        robotYSpeed = m_y;
        robotTrunSpeed = m_t;

        if(finishedD && finishedT){
            return true;
        }
        return false;
    }

    public void resetAll(){
        xController.reset();
        yController.reset();
        tethaController.reset();
    }

    public void setPidCoef_x(double p, double i, double d, double f){
        xController.changeVars(new double[]{p, i, d, f});
    }
    public void setPidCoef_y(double p, double i, double d, double f){
        yController.changeVars(new double[]{p, i, d, f});
    }
    public void setPidCoef_tetha(double p, double i, double d, double f){
        tethaController.changeVars(new double[]{p, i, d, f});
    }


    public boolean goToPositionFC(double x, double y, double tetha_deg){
        double deltaX = x - robotPose.x;
        double deltaY = y - robotPose.y;
        double m_x = xController.calculate(0, deltaX, System.currentTimeMillis());
        double m_y = yController.calculate(0, deltaY, System.currentTimeMillis());
        double deltaT =angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());
        FasterThanRR.moveFieldCentric(m_x, m_y, m_t);
        clipSpeeds(1);
        if(abs(deltaY) < 0.7 && abs(deltaX) < 0.7 && abs(toDegrees(deltaT)) < 0.5){
            resetAll();
            return true;
        }
        return false;
    }

    public boolean goToPositionFC(double x, double y, double tetha_deg, double clipMaxSpeed){
        double deltaX = x - robotPose.x;
        double deltaY = y - robotPose.y;
        double m_x = xController.calculate(0, deltaX, System.currentTimeMillis());
        double m_y = yController.calculate(0, deltaY, System.currentTimeMillis());
        double deltaT =angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());
        FasterThanRR.moveFieldCentric(m_x, m_y, m_t);
        clipSpeeds(clipMaxSpeed);
        if(abs(deltaY) < 1 && abs(deltaX) < 1 && abs(toDegrees(deltaT)) < 0.8){
            resetAll();
            return true;
        }
        return false;
    }

    public boolean goToPositionFC(double x, double y, double tetha_deg, double clipMaxSpeed, boolean clipAngular){
        double deltaX = x - robotPose.x;
        double deltaY = y - robotPose.y;
        double m_x = xController.calculate(0, deltaX, System.currentTimeMillis());
        double m_y = yController.calculate(0, deltaY, System.currentTimeMillis());
        double deltaT =angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());
        FasterThanRR.moveFieldCentric(m_x, m_y, m_t);
        clipAllSpeeds(clipMaxSpeed);
        if(abs(deltaY) < 1 && abs(deltaX) < 1 && abs(toDegrees(deltaT)) < 0.8){
            resetAll();
            return true;
        }
        return false;
    }

    public boolean goToPositionFC(double x, double y, double tetha_deg, double k_x, double k_y, double k_tetha_deg){
        double deltaX = x - robotPose.x;
        double deltaY = y - robotPose.y;
        double m_x = xController.calculate(0, deltaX, System.currentTimeMillis());
        double m_y = yController.calculate(0, deltaY, System.currentTimeMillis());
        double deltaT =angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());
        FasterThanRR.moveFieldCentric(m_x, m_y, m_t);
        if(abs(deltaY) < k_y && abs(deltaX) < k_x && abs(toDegrees(deltaT)) < k_tetha_deg){
//            Log.v("deltay", String.valueOf(deltaY));
//            Log.v("deltax", String.valueOf(deltaX));
//            Log.v("deltat", String.valueOf(deltaT));
//            Log.v("RoboAngle", String.valueOf(Math.toDegrees(robotPose.angle)));
            resetAll();
            return true;
        }
        return false;
    }


    public boolean goToPositionFC(double x, double y, double tetha_deg, double clip, double k_x, double k_y, double k_tetha_deg){
        double deltaX = x - robotPose.x;
        double deltaY = y - robotPose.y;
        double m_x = xController.calculate(0, deltaX, System.currentTimeMillis());
        double m_y = yController.calculate(0, deltaY, System.currentTimeMillis());
        double deltaT =angleWrapRad(angleWrapRad(toRadians(tetha_deg))-robotPose.angle);
        double m_t = tethaController.calculate(0, deltaT, System.currentTimeMillis());
        FasterThanRR.moveFieldCentric(m_x, m_y, m_t);
        clipSpeeds(clip);
        if(abs(deltaY) < k_y && abs(deltaX) < k_x && abs(toDegrees(deltaT)) < k_tetha_deg){
//            Log.v("deltay", String.valueOf(deltaY));
//            Log.v("deltax", String.valueOf(deltaX));
//            Log.v("deltat", String.valueOf(deltaT));
//            Log.v("RoboAngle", String.valueOf(Math.toDegrees(robotPose.angle)));
            resetAll();
            return true;
        }
        return false;
    }



}
