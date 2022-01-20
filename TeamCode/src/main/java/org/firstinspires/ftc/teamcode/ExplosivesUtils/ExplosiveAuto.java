package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ExplosiveAuto extends LinearOpMode {

    public Robot robot;

    long startTime = System.currentTimeMillis();

    // The number of milliseconds the program must run for at least before ending
    final int MINIMUM_RUNNING_MILLIS = 5000;

    @Override
    public void runOpMode() throws InterruptedException {
        log("Creating robot...");
        createRobot();
        log("Initializing...");
        initialize();
        robot.prepareForAuto();
        log("-- Awaiting start --");
        waitForStart();
        log("Starting...");
        begin();

        // Wait until program has run for the minimum millis until exiting
        while(!hasRunForMinimumTime()) {
            log("Program execution has ended before the minimum time required to run this program (" + MINIMUM_RUNNING_MILLIS + " millis) had passed. Waiting to end...");
        }
    }

    /*
        Populates the robot object.
     */
    private void createRobot() {
        robot = new Robot(hardwareMap,this);
    }

    /*
        Used to initialize any motors or servos to their proper positions, as well as activate objects such as cameras or other sensors that are not created in the Robot class. The robot will already be created and initialized upon calling.
     */
    protected abstract void initialize();

    /*
        The method called once the Start button has been pressed and the 30 second Autonomous period has begun.
     */
    protected abstract void begin() throws InterruptedException;


    public boolean hasRunForMinimumTime() {
        return System.currentTimeMillis() >= startTime+MINIMUM_RUNNING_MILLIS;
    }


    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
            showError("Could not sleep Thread!");
        }
    }

    public void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    /*
        Halt progress on the program and show an error in the telemetry
     */
    public void showError(String error) {
        telemetry.addLine("Error: " + error);
        telemetry.update();
        while(opModeIsActive()) {
            // Wait here
        }
    }


    // Custom movement

    public void driveAndArm(int driveTicks, double driveSpeed, Arm.GOAL goal) {
        boolean keepDriving = true;
        boolean keepArming = true;

        long startTime = System.currentTimeMillis();

        double initialL = robot.leftEncoder();
        double initialR = robot.rightEncoder();
        double initialHeading = robot.getHeading();

        while(keepArming||keepDriving && System.currentTimeMillis()-startTime<5000) {
            if(keepDriving) {
                keepDriving = robot.driveEncodersLoop(driveTicks, (int) initialL, (int) initialR, initialHeading, startTime, driveSpeed);
            }

            if(keepArming) {
                switch (goal) {
                    case HIGH: robot.arm.aimToHigh();
                    case MID: robot.arm.aimToMid();
                    case LOW: robot.arm.aimToLow();
                }
            }

            switch(goal) {
                case HIGH: keepArming = robot.arm.isArmCloseToEnc(robot.arm.HIGH_GOAL_ENC);
                case MID: keepArming = robot.arm.isArmCloseToEnc(robot.arm.MID_GOAL_ENC);
                case LOW: keepArming = robot.arm.isArmCloseToEnc(robot.arm.LOW_GOAL_ENC);
            }
        }
    }

}
