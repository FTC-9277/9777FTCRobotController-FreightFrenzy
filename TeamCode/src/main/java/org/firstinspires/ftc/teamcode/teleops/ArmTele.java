package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "ArmTele")
public class ArmTele extends ExplosiveTele {
    @Override
    protected void initialize() {

    }

    @Override
    protected void looping() {
        if(Math.abs(gamepad2.left_stick_y)>0.2) {
            robot.arm.setJoint0Speed(gamepad2.left_stick_y);
        } else {
            robot.arm.stop();
        }

//        if(gamepad2.a) {
//            robot.arm.openGrabber();
//        } else if (gamepad2.y) {
//            robot.arm.closeGrabber();
//        }
    }
}
