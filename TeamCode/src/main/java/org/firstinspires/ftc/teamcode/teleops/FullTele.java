package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {
    @Override
    protected void initialize() {
        robot.prepareForAuto();
    }

    boolean resetHeld = false;
    int resetNum = 0;

    @Override
    protected void looping() {
        if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
            robot.fright(0.6 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.bright(0.6 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.fleft(0.6 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.bleft(0.6 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));
        } else {
            robot.stop();
        }

        telemetry.addLine("fright:" + robot.fright.getVelocity());
        telemetry.addLine("bright:" + robot.bright.getVelocity());
        telemetry.addLine("fleft:" + robot.fleft.getVelocity());
        telemetry.addLine("bleft:" + robot.bleft.getVelocity());
        telemetry.addLine("Joint0: " + robot.arm.joint0.getCurrentPosition());
        telemetry.addLine("Joint1: " + robot.arm.joint1.getPosition());
        telemetry.update();

        if (gamepad2.left_trigger > 0.2) {
            robot.carouselMover.setPower(-gamepad1.left_trigger);
        } else if (gamepad2.right_trigger > 0.2) {
            robot.carouselMover.setPower(-gamepad1.right_trigger);
        } else {
            robot.carouselMover.setPower(0.0);
        }

        if (gamepad2.left_stick_y > 0.2 && robot.arm.joint0.getCurrentPosition() < 1505) {
            robot.arm.setJoint0Speed(gamepad2.left_stick_y * 0.4);
        } else if (gamepad2.left_stick_y < -0.2 && robot.arm.joint0.getCurrentPosition() > 0) {
            robot.arm.setJoint0Speed(gamepad2.left_stick_y * 0.4);
        } else if (robot.arm.joint0.getCurrentPosition() < 0) {
            robot.arm.PIDLoop(robot.arm.joint0, 1);
        } else if(robot.arm.joint0.getCurrentPosition() > 1510) {
            robot.arm.PIDLoop(robot.arm.joint0, 1500);
        } else {
            robot.arm.stop();
        }

        if(gamepad2.dpad_up) {
            robot.arm.PIDLoop(robot.arm.joint0,100);
        }

        if(gamepad2.b) {
            robot.arm.PIDLoop(robot.arm.joint0,1440);
            robot.arm.joint1.setPosition(0.0);
        }
        if(gamepad2.x) {
            if(Math.abs(robot.arm.joint1.getPosition()-1.0)<0.4) {
                robot.arm.PIDLoop(robot.arm.joint0, 716);
            }
            robot.arm.joint1.setPosition(1.0);
        }

        if(gamepad2.a) {
            robot.arm.intake();
        } else if (gamepad2.y) {
            robot.arm.outtake();
        } else {
            robot.arm.stopFingers();
        }

        if(resetHeld) {
            robot.arm.PIDLoop(robot.arm.joint0,500);
//            resetNum++;
        }

        if(gamepad2.left_bumper&&gamepad2.right_bumper) {
            resetHeld=true;
        } else {
            if(Math.abs(robot.arm.joint0.getCurrentPosition()-500)<5) {
//                resetNum=0;
                resetHeld=false;
            }
        }


        if(gamepad2.right_stick_y<-0.2) {
            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()+0.05);
        } else if(gamepad2.right_stick_y>0.2) {
            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()-0.05);
        }
    }
}
