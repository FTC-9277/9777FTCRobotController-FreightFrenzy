package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "Main Auto")
public class MainAuto extends ExplosiveAuto {
    @Override
    protected void initialize() {
//        robot.arm();
//        waitMillis(1000);

        // Place it into a position where it fits into the 18
        robot.arm.joint0Move(1800);
    }

    @Override
    protected void begin() throws InterruptedException {

        robot.driveEncoders(500,0.75);

        robot.autoturn(-90,2);

        robot.driveEncoders(-400,0.55);

        robot.strafe(600,0.25);

        robot.carouselMover.setVelocity(-1000);

        waitMillis(2000);

        robot.carouselMover.setVelocity(0);

        robot.autoturn(-90,2);

        robot.driveEncoders(2250,0.75);

        robot.autoturn(180,2);

        robot.driveEncoders(-200,0.75);

        robot.arm.joint0Move(7000);

//        robot.arm.openGrabber();


        waitMillis(1000);

        robot.arm.joint0Move(-3000);

        robot.driveEncoders(200,0.75);

        robot.autoturn(-90,2);

        robot.drive(1.0);

        waitMillis(750);

        robot.stop();

    }
}
