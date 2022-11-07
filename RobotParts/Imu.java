package RobotParts;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Utils.MathUtils;

public class Imu {

    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation angles;
    public static double IMU_ANGLE_RAD = 0;

    public Imu(HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        initInternalImu();
        imu.initialize(parameters);

    }

    private void initInternalImu(){
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
    }

    public void update(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        IMU_ANGLE_RAD = MathUtils.angleWrapRad(heading);
    }
}
