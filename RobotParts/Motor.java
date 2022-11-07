package RobotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class Motor {

    public DcMotorEx motor ;
    MotorConfigurationType motorType;
    DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public ExpansionHubEx eHub;
    private double currentPower;

    public Motor(String name, HardwareMap hMap, Class<?> motorType, ExpansionHubEx eHub){
        motor = hMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        this.motorType = MotorConfigurationType.getMotorType(motorType);
        this.eHub = eHub;
    }

    public Motor(String name, HardwareMap hMap, Class<?> motorType){
        motor = hMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        this.motorType = MotorConfigurationType.getMotorType(motorType);
        currentPower = 0d;
    }

    public Motor reverse(){
        this.motor.setDirection(direction.inverted());
        return this;
    }

    public Motor forword(){
        this.motor.setDirection(direction);
        return this;
    }

    public Motor breakMotor(){
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    public Motor sare(){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return this;
    }

    public Motor rwe(){
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return this;
    }

    public Motor rue(){
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return this;
    }

    public void setPower(double speed){
        Range.clip(speed, -1, 1);
        if(currentPower == speed)
            return;
        currentPower = speed;
        motor.setPower(speed);
    }

    public double getEncoderTicksRaw(){
        return motor.getCurrentPosition();
    }

    public double getEncoderTicks(){
        return this.motor.getTargetPosition();
    }

    public double getPower(){
        return motor.getPower();
    }

    public double getVelo(){
        return motor.getVelocity();
    }
}
