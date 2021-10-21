package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "Park Auto")
public class ParkAuto extends ExplosiveAuto {

    @Override
    protected void initialize() {
        
    }

    @Override
    protected void begin() throws InterruptedException {
        robot.drive(1.0);
        waitMillis(700);
        robot.stop();
    }
}
