package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "FOCTele")
public class FOCTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {

        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {

            double initX=(gamepad1.left_stick_y*0.5)*Math.sqrt(2);
            double initY=(-gamepad1.left_stick_x*0.5)*Math.sqrt(2);

            //Find the angle of these polar coordinates
            double angle = Math.atan2(initX,initY);

            //Convert to degrees
            angle = Math.toDegrees(angle);

            angle = robot.getHeading()-angle-180;

            //Convert back to radians
            angle = Math.toRadians(angle);

            double finalX=-Math.cos(angle);
            double finalY=Math.sin(angle);

            log("Final X:" + finalX + ", FinalY: " + finalY);

            double mult = Math.max(Math.abs(gamepad1.left_stick_x),Math.abs(gamepad1.left_stick_y));

            robot.fright.setPower(mult*(finalY-finalX) - gamepad1.right_stick_x);
            robot.bright.setPower(mult*(finalY+finalX) - gamepad1.right_stick_x);
            robot.fleft.setPower(mult*(finalY+finalX) + gamepad1.right_stick_x);
            robot.bleft.setPower(mult*(finalY-finalX) + gamepad1.right_stick_x);
        } else {
            robot.stop();
        }

//        if(Math.abs(gamepad2.right_trigger)>0.2) {
//            robot.shoot(-gamepad2.right_trigger);
//        } else if(Math.abs(gamepad2.left_trigger)>0.2) {
//            robot.shoot(gamepad2.left_trigger);
//        } else {
//            robot.shoot(0);
//        }

        if(gamepad2.a) {
            robot.intake();
        } else if (gamepad2.y) {
            robot.outtake();robot.outtake();
        } else {
            robot.stopIntake();
        }

        if(gamepad2.dpad_up) {
            robot.conveyor(-1);
        } else if(gamepad2.dpad_down) {
            robot.conveyor(1);
        } else {
            robot.conveyor(0);
        }

    }

//    public double rotateAngle(double init, double rotateBy) {
//        if(init+rotateBy>360) {
//            return (init+rotateBy)-(360*(Math.floor(init+rotateBy/360)));
//        }
//        if(init+rotateBy<0) {
//            return (init+rotateBy-())
//        }
//    }


}