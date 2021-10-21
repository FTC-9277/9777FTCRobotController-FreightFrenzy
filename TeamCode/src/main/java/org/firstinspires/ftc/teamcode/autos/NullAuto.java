package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;

@Autonomous(name = "Null Auto")
public class NullAuto extends ExplosiveAuto {

    @Override
    protected void initialize() {

    }

    // Just a number to display on the telemetry to show somethings happening
    int count = 0;

    @Override
    protected void begin() {
        while(!hasRunForMinimumTime()) {
            count++;
            telemetry.addData("Running null auto!", count);
            telemetry.update();
        }
    }
}
