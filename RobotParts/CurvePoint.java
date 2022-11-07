package RobotParts;


public class CurvePoint {
    public double x;
    public double y;
    public double tetha_deg;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
//        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(double x, double y, double tetha_deg, double followDistance){
        this.x = x;
        this.y = y;
        this.tetha_deg = tetha_deg;
        this.followDistance = followDistance;
    }
    public CurvePoint(CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        pointLength = thisPoint.pointLength;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
    }


    public Point toPoint(){
        return new Point(x,y);
    }

    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }
}
