package RobotParts;

public class Pose {
    public double x;
    public double y;
    public double angle;

    public Pose(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public Pose(){
        x = 0;
        y = 0;
        angle = 0;
    }
    public void resetValues(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}
