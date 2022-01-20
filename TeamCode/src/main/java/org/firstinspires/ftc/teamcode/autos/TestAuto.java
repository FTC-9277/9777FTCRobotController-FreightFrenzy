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

//        int bleftTicks = robot.bleft.getCurrentPosition();
//        int fleftTicks = robot.fleft.getCurrentPosition();
//        int brightTicks = robot.bright.getCurrentPosition();
//        int frightTicks = robot.fright.getCurrentPosition();
//        boolean keepGoing = true;
//        while(keepGoing) {
//            int changeInBleft = robot.bleft.getCurrentPosition()-bleftTicks;
//            int changeInFleft = robot.fleft.getCurrentPosition()-fleftTicks;
//            int changeInBright = robot.bright.getCurrentPosition()-brightTicks;
//            int changeInFright = robot.fright.getCurrentPosition()-frightTicks;
//
//            //If going forward change is positive
//
//            double encoderDistanceX = 0.0;
//            double encoderDistanceY = 0.0;
//
//            encoderDistanceX += changeInBleft/Math.sqrt(2);
//            encoderDistanceX += changeInFleft/Math.sqrt(2);
//            encoderDistanceX += changeInBright/Math.sqrt(2);
//            encoderDistanceX += changeInFright/Math.sqrt(2);
//
//            encoderDistanceY += changeInBleft/Math.sqrt(2);
//            encoderDistanceY += changeInFleft/Math.sqrt(2);
//            encoderDistanceY += changeInBright/Math.sqrt(2);
//            encoderDistanceY += changeInFright/Math.sqrt(2);
//
//            telemetry.addLine("Change in BLEFT: " + changeInBleft);
//            telemetry.addLine("Change in FLEFT: " + changeInFleft);
//            telemetry.addLine("Change in BRIGHT: " + changeInBright);
//            telemetry.addLine("Change in FRIGHT: " + changeInFright);
//            telemetry.addLine("Distance: " + encoderDistanceX);
//            telemetry.update();
//
//        }

//        robot.driveEncoders(500,0.35);

        robot.driveToPosition(60,30,0.075);

//        robot.autoturn(90,3);

//        robot.driveEncoders(500,0.1);
//
//        robot.autoturn(-90,3);
//
//        robot.driveEncoders(1000,0.1);
    }
}
