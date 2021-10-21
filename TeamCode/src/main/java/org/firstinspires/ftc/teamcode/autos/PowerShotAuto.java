package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "PSAuto")
public class PowerShotAuto extends ExplosiveAuto {

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

        robot.throttleDrive(0.7,1700);
        robot.turnToPS(Robot.PowerShotPos.RIGHT);

        robot.shoot(Robot.ShooterSpeed.AUTO_PS);
        waitMillis(1250);
        robot.conveyor(-1.0);
        waitMillis(300);
        robot.conveyor(0.0);

        robot.shoot(Robot.ShooterSpeed.AUTO_PS);
        robot.turnToPS(Robot.PowerShotPos.MID);
        robot.conveyor(-1.0);
        waitMillis(300);
        robot.conveyor(0.0);

        robot.shoot(Robot.ShooterSpeed.AUTO_PS_HIGH);
        robot.turnToPS(Robot.PowerShotPos.LEFT);
        robot.intake();
        robot.conveyor(-1.0);
        waitMillis(1500);
        robot.conveyor(0.0);
        robot.stopIntake();
        robot.stopShooter();

        robot.turnToAngle(0);

        //Now on depends on vision

//        value[0]=4;

        if(value[0] == 0) {
            robot.throttleDrive(1.0,700);
            robot.turnToAngle(90);
            robot.throttleDrive(1.0,200);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.throttleDrive(1.0,480);

            robot.dropWobbler();
            robot.openGrabber();

            robot.turnToAngle(0);

            robot.throttleDrive(-0.7,1000);
            robot.turnToAngle(4);
            robot.throttleDrive(-0.5,625);
            robot.turnToAngle(0);

            waitMillis(1000);

            robot.closeGrabber();
            waitMillis(250);

            robot.liftWobbler();
            waitMillis(250);

            robot.throttleDrive(1.0,1200);

            robot.turnToAngle(90);

            robot.throttleDrive(-1.0,370);

            robot.dropWobbler();
            waitMillis(1000);
            robot.openGrabber();
            waitMillis(250);
            robot.liftWobbler();

            robot.throttleDrive(2.0,500);


        } else if (value[0]==1) {
            robot.throttleDrive(0.5,850);
            robot.strafeEncoders(0.25, 500, Robot.Direction.LEFT);
            robot.turnToAngle(180);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.turnToAngle(0);

            robot.closeGrabber();

            robot.intake();
            waitMillis(500);
            robot.throttleDrive(-0.5,1500);

            robot.conveyor(-1.0);
            robot.shoot(Robot.ShooterSpeed.TELEOP);

            robot.throttleDrive(0.8,100);
            robot.turnToAngle(0);

            waitMillis(2000);

            robot.stopShooter();
            robot.stopIntake();
            robot.conveyor(0.0);

            robot.throttleDrive(1.0,1000);
        } else if (value[0] == 4) {
            robot.throttleDrive(1.0,2000);
            robot.strafeEncoders(0.5, 300, Robot.Direction.LEFT);
            robot.turnToAngle(90);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.turnToAngle(0);

            robot.closeGrabber();

            robot.strafeEncoders(0.5, 100, Robot.Direction.LEFT);

//            robot.strafeEncoders(0.5,50, Robot.Direction.LEFT);

            //Go to collect rings

            robot.throttleDrive(-1.0,2000);

            robot.shoot(Robot.ShooterSpeed.FULL_FORWARD);
            robot.intake();
            robot.conveyor(-1.0);

            robot.throttleDrive(-0.4,1200);

            robot.turnToAngle(4);

            robot.throttleDrive(0.5,400);
            robot.turnToAngle(4);

            waitMillis(1500);

            robot.stopShooter();
            robot.stopIntake();
            robot.conveyor(0.0);

            robot.throttleDrive(1.0,1000);
        }

    }

    public void shootPS() {
    }
}
