package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

@TeleOp(name = "Servo Test")
public class ServoTest extends ExplosiveTele {
    @Override
    protected void initialize() {
//        robot.arm.openGrabber();
    }

    @Override
    protected void looping() {

        telemetry.addLine(robot.arm.fingers.getPower()+"");
        telemetry.update();

        if(gamepad1.y) {
//            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()+0.0005);
            robot.arm.intake();
        } else if (gamepad1.a) {
//            robot.arm.joint1.setPosition(robot.arm.joint1.getPosition()-0.0005);\
            robot.arm.outtake();
        } else {
            robot.arm.stopFingers();
        }
    }
}
