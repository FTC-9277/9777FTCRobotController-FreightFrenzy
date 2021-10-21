package org.firstinspires.ftc.teamcode.ExplosivesUtils;

public class PIDController {

    double kP,kI,kD;

    public PIDController(double kP, double kI, double kD) {
        this.kP=kP;
        this.kI=kI;
        this.kD=kD;
    }

    // Resets all values to do new calculations entirely
    public void reset() {
        startTime=0;
        previousError=0.0;
        previousTime=0;
    }

    long startTime=0;
    long previousTime=0;

    double previousError=0.0;

    double p,i,d;

    public double calculate(double error) {
        if(startTime==0) {
            startTime=System.currentTimeMillis();
            previousError=error;
            previousTime=startTime;
        }

        long currentTime = System.currentTimeMillis();

        p = kP * error;

        i += kI * (error * (currentTime-previousTime));

        d = kD * (error - previousError) / (currentTime - previousTime);

        previousError = error;
        previousTime = currentTime;

        return p;

    }

}
