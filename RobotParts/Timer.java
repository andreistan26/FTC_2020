package RobotParts;

public class Timer {
    long last_time;
    long current_time;
    long goal_time;
    boolean started;
    long timeDelta;

    public Timer(long timeMs){
        started = false;
        timeDelta = timeMs;
    }

    public void start(){
        current_time = System.currentTimeMillis();
        goal_time = current_time;
        last_time = current_time;
        goal_time+=timeDelta;
        started = true;
    }

    public void setTimeDelta(long timeMs){
        timeDelta = timeMs;
    }

    public boolean update(){
        if(started){
            start();
        }
        current_time = System.currentTimeMillis();
        if(current_time > goal_time){
            stop();
            return true;
        }
        return false;
    }

    public void stop(){
        started = false;
    }

    public void restart(){
        started = false;

    }


}
