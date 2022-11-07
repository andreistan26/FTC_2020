package test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RobotParts.BaseOpMode;

@TeleOp
public class AutoFeedTest extends BaseOpMode {
    private boolean autoFeed;
    int ringsRemaining = 3;
    @Override
    public void onMainLoop() {
         robot.updateTeleop();
         if(avxGamepad.b.value) autoFeed = true;
         if(avxGamepad.x.value) autoFeed = false;
         if(autoFeed){
             if(ringsRemaining != 0){
                 if(robot.buttonAction.flick_auto()){
                     ringsRemaining--;
                 }
             }
         }else {
             ringsRemaining = 3;
    }
    }

    @Override
    public void initLoop() {
        autoFeed = false;

    }
}
