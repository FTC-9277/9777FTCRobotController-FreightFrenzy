package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    public final double DAMPING = 0.5;

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_y) > 0.2) {
            robot.drive(gamepad1.left_stick_y*DAMPING);
        } else if(Math.abs(gamepad1.right_stick_x) > 0.2) {
            robot.turn(-gamepad1.right_stick_x*DAMPING);
        } else {
            robot.drive(0);
        }


    }
}
