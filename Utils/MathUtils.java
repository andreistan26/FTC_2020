package Utils;

import static java.lang.Math.*;
import RobotParts.Point;

public class MathUtils {

    public static final double TAU = Math.PI*2;

    public static double angleWrapRad(double angle){

        while(angle>Math.PI){
            angle -= TAU;
        }

        while(angle< -Math.PI){
            angle += TAU;
        }

        return angle;
    }

    public static double angleWrapDeg(double angle){
        while(angle>180){
            angle -= 360.0;
        }
        while(angle<-180){
            angle += 360.0;
        }

        return angle;
    }

    public static Point pointDelta(Point robotDelta, double heading){
        double c = cos(heading);
        double s = sin(heading);
        double x = robotDelta.x;
        double y = robotDelta.y;

        double newY = y*c - x*s;
        double newX = y*s + x*c;

        return new Point(newX, newY);
    }
}
