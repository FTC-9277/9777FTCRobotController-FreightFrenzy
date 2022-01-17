package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "TestAuto")
public class TestAuto extends ExplosiveAuto {
    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {
        robot.driveEncoders(500,0.35);

        robot.driveToPosition(60,30,0.35);

//        robot.autoturn(90,3);

//        robot.driveEncoders(500,0.1);
//
//        robot.autoturn(-90,3);
//
//        robot.driveEncoders(1000,0.1);
    }
}
