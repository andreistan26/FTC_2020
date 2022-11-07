package Utils.VisionEasyOpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled
public class opencvSkystoneDetector extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private static int valMid = -1;
    private static int valTop = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f;
    private static float offsetY = 0f;

    private static float[] midPos = {4.97f/8f+offsetX, 5.3f/8f+offsetY};
    private static float[] topPos = {4.97f/8f+offsetX, 3.3f/8f+offsetY};

    private final int rows = 800;
    private final int cols = 600;

    OpenCvCamera cvCamera;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "logitech"), cameraMonitorViewId);
        cvCamera.openCameraDevice();
        cvCamera.setPipeline(new StageSwitchingPipeline());
        cvCamera.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valMid/255+"   "+valTop/255);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);


        }
    }


    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {/
            detection,
            THRESHOLD,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {


            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object


            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixTop = thresholdMat.get((int)(input.rows()* topPos[1]), (int)(input.cols()* topPos[0]));//gets value at circle
            valTop = (int)pixTop[0];


            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointTop = new Point((int)(input.cols()* topPos[0]), (int)(input.rows()* topPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointTop,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(topPos[0]-rectWidth/2),
                            input.rows()*(topPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(topPos[0]+rectWidth/2),
                            input.rows()*(topPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
