package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.Arm;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "DuckHighGoal Auto")
public class MainAuto extends ExplosiveAuto {
    @Override
    protected void initialize() {
        robot.prepareForAuto();
    }

    @Override
    protected void begin() throws InterruptedException {

//        while(Math.abs(robot.arm.joint0.getCurrentPosition()-robot.arm.HIGH_GOAL_ENC)>5) {
//            robot.arm.PIDLoop(robot.arm.joint0,robot.arm.HIGH_GOAL_ENC);
//        }

        robot.arm.setArmPosition(robot.arm.HIGH_GOAL_ENC);

        robot.arm.intake();

        robot.driveEncoders(725,0.15);

        robot.arm.outtake();

        long t = System.currentTimeMillis();
        while(System.currentTimeMillis()-t<2000) {
            robot.arm.aimToPosition();
        }

        robot.arm.setArmPosition(100);
        robot.arm.stopFingers();

        robot.autoturn(110,2);
        robot.driveEncoders(2300,0.5);

        robot.arm.setArmPosition(0);

        robot.bleft(-0.2);
        robot.fleft(0.2);
        robot.bright(0.2);
        robot.fright(-0.2);
        t = System.currentTimeMillis();
        while(System.currentTimeMillis()-t<1200) {
            robot.arm.aimToPosition();
        }

        robot.stop();

        robot.carouselMover.setVelocity(-800);

        waitMillis(3000);

        robot.carouselMover.setVelocity(0);

        robot.drive(0.2);
        t = System.currentTimeMillis();
        while(System.currentTimeMillis()-t<500) {
            robot.arm.aimToPosition();
        }

        robot.bleft(0.4);
        robot.fleft(-0.4);
        robot.bright(-0.4);
        robot.fright(0.4);
        t = System.currentTimeMillis();
        while(System.currentTimeMillis()-t<700) {
            robot.arm.aimToPosition();
        }

        robot.drive(0.2);
        t = System.currentTimeMillis();
        while(System.currentTimeMillis()-t<500) {
            robot.arm.aimToPosition();
        }

    }


}
