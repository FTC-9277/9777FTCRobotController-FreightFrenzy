package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "ServoProgramming")
public class ServoProgramming extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    double wobblerPos=0.6;

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_y)>0.2) {
            wobblerPos+=gamepad1.left_stick_y/4000;
            telemetry.addLine("pos: " + wobblerPos);
            telemetry.update();
            robot.wobbler.setPosition(wobblerPos);
        }
    }
}
