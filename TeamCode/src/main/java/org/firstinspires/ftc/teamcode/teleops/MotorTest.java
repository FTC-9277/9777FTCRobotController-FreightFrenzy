package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "Motor Test Tele")
public class MotorTest extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {
        if(gamepad1.a) {
            robot.bleft.setVelocity(1000000);
        } else {
            robot.bleft.setVelocity(0);
        }

        telemetry.addLine(""+robot.bleft.getVelocity());
        telemetry.update();
    }
}
