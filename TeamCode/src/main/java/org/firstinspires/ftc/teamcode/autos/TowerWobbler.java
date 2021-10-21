package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "TowerWobbler")
public class TowerWobbler extends ExplosiveAuto {

    Camera camera;
    Sampler sampler;

    @Override
    protected void initialize() {
        log("Starting.");
        camera = new Camera(hardwareMap,false);
        sampler = new Sampler(camera);
        log("Initialized!");
    }

    @Override
    protected void begin() throws InterruptedException {
        log("cycling");
        camera.cycle();
        log("end");
        log("sampling");
        double[] value = sampler.sample();
        log(("Sample: " + value[0] + ", TotalW: " + value[1] + ", HeightUp: " + value[2] + ", HeightDown: " + value[3] + ", W: " + value[4] + ", H: " + value[5] + ", center: (" + value[6] + "," + value[7] + ")"));

        robot.throttleDrive(1.0, 1000);

        waitMillis(150);

        robot.strafeEncoders(1.0, 700, Robot.Direction.LEFT);

        robot.turnToAngle(0);

        robot.shoot(Robot.ShooterSpeed.AUTO_TOWERGOAL);
        waitMillis(2000);

        robot.conveyor(-1.0);
        robot.intake();
        waitMillis(3000);

        robot.stop();
        waitMillis(1000);

        robot.stopIntake();
        robot.conveyor(0);
        robot.shoot(Robot.ShooterSpeed.STOP);

        waitMillis(250);
        if(value[0]==0) {

            robot.throttleDrive(1.0, 600);

            robot.turnToAngle(90);

            waitMillis(500);

            robot.dropWobbler();
            waitMillis(1000);

            robot.openGrabber();
            waitMillis(500);

            robot.liftWobbler();
            waitMillis(500);
            robot.closeGrabber();

            robot.stop();

        } else if(value[0]==1) {

            robot.throttleDrive(1.0, 600);

            robot.turnToAngle(180);

            waitMillis(500);

            robot.dropWobbler();
            waitMillis(1000);

            robot.openGrabber();
            waitMillis(500);

            robot.liftWobbler();
            waitMillis(500);
            robot.closeGrabber();

            robot.stop();
        } else {
            robot.throttleDrive(1.0, 1000);

            robot.turnToAngle(90);

            waitMillis(500);

            robot.dropWobbler();
            waitMillis(1000);

            robot.openGrabber();
            waitMillis(500);

            robot.liftWobbler();
            waitMillis(500);
            robot.closeGrabber();

            robot.stop();

            robot.turnToAngle(0);

            robot.driveBackwardEncoders(1.0,500);

        }
    }
}
