package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@TeleOp(name = "PSTest")
public class PowerShotTest extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    double right=21;
    double mid=26.5;
    double left=30.5;

    @Override
    protected void looping() {
        if(gamepad1.y) {
            robot.turnToAngle(mid);
        } else if (gamepad1.b) {
            robot.turnToAngle(right);
        } else if(gamepad1.x) {
            robot.turnToAngle(left);
        }

        if(Math.abs(gamepad2.right_trigger)>0.2) {
            robot.shoot(Robot.ShooterSpeed.AUTO_PS);
        } else if(Math.abs(gamepad2.left_trigger)>0.2) {
            robot.shoot(Robot.ShooterSpeed.FULL_BACKWARD);
        } else {
            robot.stopShooter();
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
