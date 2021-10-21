package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "StrafeTest")
public class StrafeTestTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2) {
            robot.fright.setPower(gamepad1.left_stick_y);
            robot.fleft.setPower(gamepad1.right_stick_y);
            robot.bright.setPower(gamepad1.right_stick_y);
            robot.bleft.setPower(gamepad1.left_stick_y);
        } else {
            robot.stop();
        }
    }
}
