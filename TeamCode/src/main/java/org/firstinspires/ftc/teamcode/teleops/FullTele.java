package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
            robot.fright(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.bright(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.fleft(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.bleft(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) - gamepad1.right_stick_x));
        } else {
            robot.stop();
        }

        telemetry.addLine("fright:"+ robot.fright.getVelocity());
        telemetry.addLine("bright:"+ robot.bright.getVelocity());
        telemetry.addLine("fleft:"+ robot.fleft.getVelocity());
        telemetry.addLine("bleft:"+ robot.bleft.getVelocity());
        telemetry.addLine("Joint0: "+robot.arm.joint0.getCurrentPosition());
        telemetry.update();

        if(gamepad1.left_trigger > 0.2) {
            robot.carouselMover.setPower(-gamepad1.left_trigger);
        } else if(gamepad1.right_trigger > 0.2) {
            robot.carouselMover.setPower(-gamepad1.right_trigger);
        } else {
            robot.carouselMover.setPower(0.0);
        }

        if(Math.abs(gamepad2.left_stick_y)>0.2) {
            robot.arm.setJoint0Speed(gamepad2.left_stick_y*0.2);
        } else {
            robot.arm.stop();
        }

        if(gamepad2.a) {
            robot.arm.intake();
        } else if (gamepad2.y) {
            robot.arm.outtake();
        } else {
            robot.arm.stopFingers();
        }


        if(gamepad2.right_stick_y<-0.2) {
            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()+0.05);
        } else if(gamepad2.right_stick_y>0.2) {
            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()-0.05);
        }
    }
}
