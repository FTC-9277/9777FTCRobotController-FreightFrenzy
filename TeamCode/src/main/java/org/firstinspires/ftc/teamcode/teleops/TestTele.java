package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "TestTele")
public class TestTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {
        if(gamepad2.y) {
            robot.intake();
        } else if (gamepad2.a) {
            robot.outtake();
        } else {
            robot.stopIntake();
        }
    }
}
