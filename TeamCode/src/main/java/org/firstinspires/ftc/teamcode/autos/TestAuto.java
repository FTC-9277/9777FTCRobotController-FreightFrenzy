package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "Test Auto")
public class TestAuto extends ExplosiveAuto {

    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {

        robot.throttleDrive(1.0,1750);

    }
}
