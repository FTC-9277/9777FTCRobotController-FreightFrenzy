package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "DriveVelocityCalibration")
public class DriveVelocityCalibration extends ExplosiveTele {
    @Override
    protected void initialize() {
        robot.resetEncoders();
    }

    double velocity=0;

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_y)>0.2) {
            velocity+=gamepad1.left_stick_y;
        }
        telemetry.addLine("Velocity: " + velocity);
        telemetry.addLine("fright: " + robot.fright.getVelocity());
        telemetry.addLine("bright: " + robot.bright.getVelocity());
        telemetry.addLine("fleft: " + robot.fleft.getVelocity());
        telemetry.addLine("bleft: " + robot.bleft.getVelocity());
        telemetry.update();
        robot.bleft.setVelocity(velocity);
        robot.bright.setVelocity(velocity);
        robot.fleft.setVelocity(velocity);
        robot.fright.setVelocity(velocity);
    }
}
