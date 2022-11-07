package RobotParts;



import com.qualcomm.robotcore.util.MovingStatistics;

import java.util.ArrayList;

import static java.lang.Math.abs;

public class PIDFController {
    private double kP, kI, kD, kF;
    private double kT;
    private double sp;
    private double mv;
    private boolean use_kV;
    private double last_power;

    public double targetVelocity;
    public double targetAcceleration;
    private double kV, kA, kStatic;
    private boolean use_kt;

    private double error;
    private double lastError = 0, errorSum = 0, lastTime = 0;


    public PIDFController(double[] pidf){
        this(pidf,0,0,0);
    }

    /**
     * @param pidf coeficintii controlerului
     * @param sp punctul in care trebuie sa ajungem
     * @param mv punctul in care ne aflam
     * @param kT marja de eroare a controlarului
     */

    public PIDFController(double[] pidf, double sp, double mv, double kT){
        this.kP = pidf[0];
        this.kI = pidf[1];
        this.kD = pidf[2];
        this.kF = pidf[3];

        this.sp = sp;
        this.mv = mv;
        this.kT = kT;

        this.error = sp-mv;

    }


    public void set_kv(double kV){
        use_kV = true;
        this.kV = kV;
    }

    public void setPositions(double mv, double sp){
        this.mv = mv;
        this.sp = sp;
    }

    public void changeVars(double[] pidf){

        this.kP = pidf[0];
        this.kI = pidf[1];
        this.kD = pidf[2];
        this.kF = pidf[3];
    }

    public void setThreshold(double kT){
        use_kt = true;
        this.kT = kT;
    }

    public void resetIntegral(){
        errorSum = 0;
    }

    public void reset(){
        errorSum = 0;
        error = 0;
        lastError = 0;
        lastTime = 0;
        last_power = 0;
    }

    public double calculate(double mv, double sp, double time){
        double dt;
        error = sp-mv;
        if(abs(error) <= kT && use_kt) {
            resetIntegral();
            return 0;
        }
        double change = 0;
        double p = error*kP;

        if(time != 0) {
            if(lastTime == 0){
                lastTime = time;
            }
            dt = (time - lastTime)/1000;
        }else{
            dt = 0.02;
        }
        errorSum += ((error + lastError)/2)*dt;
        double i = kI*errorSum;

        if(dt !=0)
            change = (error - lastError) / dt;
        double d = change*kD;


        lastTime = time;
        lastError = error;
        if(use_kV)
            return p+i+d+kF+kV*sp;
        if(error > 0){
            return p+i+d+kF;
        }else {
            return p + i + d - kF;
        }
    }


    public double calculateShooter(double mv, double sp, double time){
        double dt;
        error = sp-mv;
        if(abs(error) <= kT && use_kt) {
            return last_power;
        }
        double change = 0;
        //P
        double p = error*kP;

        //I
        if(time != 0) {
            if(lastTime == 0){
                lastTime = time;
            }
            dt = (time - lastTime)/1000;
        }else{
            dt = 0.02;
        }
        errorSum += ((error + lastError)/2)*dt;
        double i = kI*errorSum;

        //D
        if(dt !=0)
            change = (error - lastError) / dt;
        double d = change*kD;


        lastTime = time;
        lastError = error;
        if(use_kV)
            return p+i+d+kF+kV*sp;
        if(error > 0){
            last_power = p+i+d+kF;
            return p+i+d+kF;
        }else {
            last_power = p+i+d-kF;
            return last_power;
        }
    }


}
