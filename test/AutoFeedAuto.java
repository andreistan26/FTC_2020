package test;

import android.util.Log;

import RobotParts.BaseOpMode;
import RobotParts.ButtonAction;
import RobotParts.Flicker;

import static RobotParts.RobotPosition.robotPose;

public class AutoFeedAuto extends BaseOpMode {



    boolean[] shot = {false, false, false, false, false, false};

    @Override
    public void initLoop() {

        Flicker.DELAY = 230;
    }

    @Override
    public void onMainLoop() {
        robot.updateAuto();
        if(!shot[1]){
            if (robot.buttonAction.flick_auto()) {
                Log.v("AUTO_FirstSHOT_High", "DONE" + " angle: " + Math.toDegrees(robotPose.angle));
                shot[1] = true;
            }
        } else{
                if (!shot[2]) {
                    if (robot.buttonAction.flick_auto()) {
                        shot[2] = true;
                    }
                } else {
                        if (!shot[3]) {
                            if (robot.buttonAction.flick_auto()) {
                                shot[3] = true;
                            }
                        } else {
                                if (!shot[4]) {
                                    if (robot.buttonAction.flick_auto()) {
                                        shot[4] = true;
                                    }
                                }
                            }
                        }
                    }
                }
}
