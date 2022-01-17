package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "PID Adjust")
public class PIDAdjust extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    public final double DAMPING = 0.5;

    public Boolean locked = false;

    public double P = 10.0;

    public Boolean locked2=false;

    @Override
    protected void looping() {

        if(gamepad1.y&&!locked) {
            locked = true;
            P+=0.2;
            robot.right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,3.0,0.0,0.0));
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(gamepad1.a&&!locked) {
            locked=true;
            P-=0.2;
            robot.right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,3.0,0.0,0.0));
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(!gamepad1.a&&!gamepad1.y) {
            locked=false;
        }

        if(gamepad1.b) {
            robot.drive(1);
            locked2=false;
        } else if (!locked2) {
            locked2=true;
            robot.drive(-1);
            waitMillis(100);
            robot.drive(0);
        }

//        if(Math.abs(gamepad1.left_stick_y) > 0.2) {
//            robot.drive(gamepad1.left_stick_y*DAMPING);
//        } else if(Math.abs(gamepad1.right_stick_x) > 0.2) {
//            robot.turn(-gamepad1.right_stick_x*DAMPING);
//        } else {
//            robot.drive(0);
//        }


    }
}
