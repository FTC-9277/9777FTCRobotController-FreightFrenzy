package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@Autonomous(name = "Shoot")
public class ShootAuto extends ExplosiveAuto {
    @Override
    protected void initialize() {

    }

    @Override
    protected void begin() throws InterruptedException {

        robot.throttleDrive(0.6,1100);

        waitMillis(500);

        robot.turnToAngle(90);

        waitMillis(500);

        robot.throttleDrive(0.6,150);
        robot.stop();

        waitMillis(500);

        robot.turnToAngle(0);

        robot.shoot(Robot.ShooterSpeed.AUTO_TOWERGOAL);
        waitMillis(2000);

        robot.conveyor(-1.0);
        robot.intake();
        waitMillis(5000);

        robot.stop();
        waitMillis(1000);

        robot.stopIntake();
        robot.conveyor(0);
        robot.shoot(Robot.ShooterSpeed.STOP);

        waitMillis(250);

        robot.throttleDrive(0.6,300);

        robot.turnToAngle(180);

        waitMillis(500);

        robot.turn(0.3, Robot.Direction.LEFT);
        waitMillis(250);

        robot.stop();

        waitMillis(500);

        robot.dropWobbler();
        waitMillis(1000);

        robot.openGrabber();
        waitMillis(500);

        robot.liftWobbler();
        waitMillis(500);
        robot.closeGrabber();

        robot.stop();

    }
}
