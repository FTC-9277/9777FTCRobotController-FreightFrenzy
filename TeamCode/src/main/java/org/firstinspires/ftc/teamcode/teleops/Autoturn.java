package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@TeleOp(name = "Autoturn")
public class Autoturn extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    double angle=15.0;

    @Override
    protected void looping() {
        if(Math.abs(gamepad1.left_stick_x)>0.2) {
            angle+=gamepad1.left_stick_x*0.05;
            telemetry.addLine("Angle: " + angle);
            telemetry.update();
        }



        if(gamepad1.y) {
            robot.turnToAngle(angle);
            telemetry.addLine("Done!");
            telemetry.update();
        }

        if(Math.abs(gamepad2.right_trigger)>0.2) {
            robot.shoot(Robot.ShooterSpeed.FULL_FORWARD);
        } else if(Math.abs(gamepad2.left_trigger)>0.2) {
            robot.shoot(Robot.ShooterSpeed.FULL_BACKWARD);
        } else {
            robot.shoot(Robot.ShooterSpeed.STOP);
        }

        if(gamepad2.a) {
            robot.intake();
        } else if (gamepad2.y) {
            robot.outtake();
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
}
