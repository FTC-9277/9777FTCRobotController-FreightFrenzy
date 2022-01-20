package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    HardwareMap hardwareMap;
    OpMode opMode;

    public double getJ0() {
        return j0;
    }

    public double getJ1() {
        return j1;
    }

    public double getJ2() {
        return j2;
    }

    private double JOINT0_INITIAL = 0.5;
    private double JOINT1_INITIAL = 0.5;
    private double JOINT2_INITIAL = 0.0;
    private double j0 = JOINT0_INITIAL;
    private double j1 = JOINT1_INITIAL;
    private double j2 = JOINT2_INITIAL;

//    public DcMotorEx joint0,joint1;
//    public Servo joint2,joint3;

    public DcMotorEx joint0;
    public Servo joint1;
    public CRServo fingers;

    public Arm(HardwareMap hardwareMap, OpMode op) {
        this.hardwareMap = hardwareMap;
        opMode = op;

        joint0 = hardwareMap.get(DcMotorEx.class,"joint0");
        joint0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        joint1 = hardwareMap.get(Servo.class,"joint1");

        fingers = hardwareMap.get(CRServo.class,"fingers");

//        joint0 = hardwareMap.get(DcMotorEx.class,"arm0");
//        joint1 = hardwareMap.get(DcMotorEx.class,"arm1");
//        joint2 = hardwareMap.get(Servo.class, "joint2");
//        joint3 = hardwareMap.get(Servo.class, "joint3");
//
//        joint0.setVelocityPIDFCoefficients(1.4,4,-30,200);
//        joint1.setVelocityPIDFCoefficients(1.4,2.5,0.0,200);
//
////        joint0.setTargetPosition(-30);
////        joint1.setTargetPosition(50);
//
//        joint0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        joint0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        joint0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        joint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public int getJoint0Position() {
        return joint0.getCurrentPosition();
    }

    public void joint0Move(int ticks) {
        ticks = -ticks;
        int initial = getJoint0Position();
        int target = initial+ticks;
        if(ticks<0) {
            setJoint0Speed(-1);
            while(getJoint0Position()>target) {
                opMode.telemetry.addLine(""+getJoint0Position());
                opMode.telemetry.addLine(""+target);
                opMode.telemetry.update();
            }
        } else {
            setJoint0Speed(1);
            while(getJoint0Position()<target) {
                opMode.telemetry.addLine(""+getJoint0Position());
                opMode.telemetry.addLine(""+target);
                opMode.telemetry.update();
            }
        }
        setJoint0Speed(0);
    }

    public void moveJoint0ToPos(int position) {
        while(Math.abs(joint0.getCurrentPosition()-position)>5) {
            PIDLoop(joint0,position);
        }
    }

    public int MAX_VELOCITY =3000;

    public void PIDLoop(DcMotorEx motor, double target) {
        double current = motor.getCurrentPosition();

        int sign=1;
        if(target<current) {
            sign=-1;
        }

        double diff = Math.abs(target-current);

        double velocity = 1;

        if(diff<100) {
            velocity = diff/100*0.6;
        } else if(diff<500) {
            velocity=0.6;
        }

        if(diff<20) {
            velocity=0.1;
        }


        //1130 0.15
        if(velocity>1) {
            velocity=1;
        } else if (velocity<-1) {
            velocity=-1;
        }

        if(velocity==0) {
            velocity=0.001;
        }

        motor.setVelocity(sign*MAX_VELOCITY*velocity);
//        motor.setPower(sign*velocity);

//        opMode.telemetry.addLine("Actual Velocity: " + motor.getVelocity());
//        opMode.telemetry.addLine("Velocity: " + velocity);
//        opMode.telemetry.addLine("Current: " + current);
//        opMode.telemetry.addLine("Target Enc: " + target);
//        opMode.telemetry.update();

    }
//
//    public void setJoint0(double val) {
//        j0=val;
//    }

    public void lowerGrabber() {
        joint1.setPosition(1.0);
    }

    public void raiseGrabber() {
        joint1.setPosition(0.0);
    }

    public void intake() {
        fingers.setPower(1.0);
    }

    public void outtake() {
        fingers.setPower(-1.0);
    }

    public void stopFingers() {
        fingers.setPower(0.0);
    }

    public final int HIGH_GOAL_ENC = 716;
    public final int MID_GOAL_ENC = 1111;
    public final int LOW_GOAL_ENC = 1367;

    public enum GOAL {
        HIGH,
        MID,
        LOW
    }

    public boolean isArmCloseToEnc(int enc) {
        return Math.abs(joint0.getCurrentPosition()-enc)<10;
    }

    //Aims the arm at the high goal
    public void aimToHigh() {
        PIDLoop(joint0,HIGH_GOAL_ENC);
        joint1.setPosition(1.0);
    }

    //Aims the arm at the mid goal
    public void aimToMid() {
        PIDLoop(joint0,MID_GOAL_ENC);
        joint1.setPosition(0.55);
    }

    //Aims the arm at the low goal
    public void aimToLow() {
        PIDLoop(joint0,LOW_GOAL_ENC);
        joint1.setPosition(0.0);
    }

    int MAX_JOINT0_VELOCITY = 3000;
    public void setJoint0Speed(double speed) {
        joint0.setVelocity(map(MAX_JOINT0_VELOCITY,speed));
        opMode.telemetry.addLine("Joint0 Velocity: "+joint0.getVelocity());
    }

    public void stop() {
        joint0.setVelocity(0);
    }

    private static double map(double targetUp, double value) {
        return targetUp*value;
    }

    private double position = -1;

    public void stopUsingPosition() {
        position = -1;
    }

    public void setArmPosition(int pos) {
        position = pos;
    }

    public void aimToPosition() {
        if(position==-1) { return; }

        PIDLoop(joint0,position);

    }

}
