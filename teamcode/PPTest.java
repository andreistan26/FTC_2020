package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import RobotParts.BaseOpMode;
import RobotParts.CurvePoint;
import RobotParts.PidDriveAuto;
import RobotParts.PurePursuit;

import static RobotParts.RobotPosition.robotPose;
@Autonomous
public class PPTest extends BaseOpMode {
    ArrayList<CurvePoint> path;
    PidDriveAuto pidDriveAuto;
    enum States{
        GO_TO_POWERSHOT, STOP
    }
    States state;
    @Override
    public void onMainLoop() {
        switch (state){
            case GO_TO_POWERSHOT: {
                break;
            }
        }


        robot.updateAuto();
        telemetry.addData("X pos", robotPose.x);
        telemetry.addData("Y pos", robotPose.y);
        telemetry.addData("Angle", Math.toDegrees(robotPose.angle));
        telemetry.update();
    }

    @Override
    public void initLoop() {
        state = States.GO_TO_POWERSHOT;
        path = new ArrayList<>();
        path.add(new CurvePoint(0.0, 150.0, 0.0,20.0));
        path.add(new CurvePoint(120.0, 120.0, 0.0,20.0));
        path.add(new CurvePoint(120.0, 60.0, 179.0,20.0));
        path.add(new CurvePoint(120.0, 40.0, 179.0,20.0));

        PurePursuit.addLastPoint(path);

        pidDriveAuto = new PidDriveAuto();
        telemetry.addData("done", "");
        telemetry.update();
    }
}
