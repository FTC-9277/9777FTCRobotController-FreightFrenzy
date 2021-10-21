package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "Motor Test Auto")
public class MotorTestAuto extends ExplosiveAuto {

    Robot robot;

    @Override
    protected void initialize() {
        robot = new Robot(hardwareMap,this);
    }

    // Just a number to display on the telemetry to show somethings happening
    int count = 0;

    @Override
    protected void begin() {

        robot.fleft.setPower(1.0);

        sleep(2000);

        robot.fleft.setPower(-1.0);

        sleep(2000);

        robot.fleft.setPower(0.0);

        while(!hasRunForMinimumTime()) {
            count++;
            telemetry.addData("Running null auto!", count);
            telemetry.update();
        }
    }
}
