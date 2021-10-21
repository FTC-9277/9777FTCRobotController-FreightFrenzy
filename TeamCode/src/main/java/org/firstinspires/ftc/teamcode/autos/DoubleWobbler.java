package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "DoubleWobbler")
public class DoubleWobbler extends ExplosiveAuto {
    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {

        robot.throttleDrive(1.0,1100);

        waitMillis(250);

        robot.turnToAngle(90);

        waitMillis(250);

        robot.throttleDrive(0.6,150);
        robot.stop();

        waitMillis(250);

        robot.turnToAngle(0);

        robot.shoot(Robot.ShooterSpeed.AUTO_TOWERGOAL);
        waitMillis(2000);

        robot.conveyor(-1.0);
        robot.intake();
        waitMillis(3000);

        robot.stop();
        waitMillis(250);

        robot.stopIntake();
        robot.conveyor(0);
        robot.shoot(Robot.ShooterSpeed.STOP);

        waitMillis(250);

        robot.throttleDrive(1.0,300);

        robot.turnToAngle(180);

        waitMillis(250);

        robot.turn(0.3, Robot.Direction.LEFT);
        waitMillis(250);

        robot.stop();

        waitMillis(250);

        robot.dropWobbler();
        waitMillis(1000);

        robot.openGrabber();
        waitMillis(500);

        robot.liftWobbler();
        waitMillis(500);
        robot.closeGrabber();

        robot.stop();

        robot.turnToAngle(5);

        waitMillis(250);

        robot.driveBackwardEncoders(1.0,1000);

        waitMillis(500);


    }
}
