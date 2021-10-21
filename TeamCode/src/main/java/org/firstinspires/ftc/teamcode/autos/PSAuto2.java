package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "PSAuto2")
public class PSAuto2 extends ExplosiveAuto {

    Camera camera;
    Sampler sampler;

    @Override
    protected void initialize() {
        log("Starting.");
//        camera = new Camera(hardwareMap,false);
//        sampler = new Sampler(camera);
        log("Initialized!");
    }

    @Override
    protected void begin() throws InterruptedException {

//        log("cycling");
//        camera.cycle();
//        log("end");
//        log("sampling");
//        double[] value = sampler.sample();
//        log(("Sample: " + value[0] + ", TotalW: " + value[1] + ", HeightUp: " + value[2] + ", HeightDown: " + value[3] + ", W: " + value[4] + ", H: " + value[5] + ", center: (" + value[6] + "," + value[7] + ")"));

        robot.throttleDrive(0.5,1350);
        robot.turnToAngle(0);

        robot.shoot(Robot.ShooterSpeed.TELEOP);
        waitMillis(1000);
        robot.conveyor(-1.0);
        waitMillis(100);
        robot.strafe(0.25, Robot.Direction.LEFT);
        waitMillis(750);
        robot.strafe(0.0, Robot.Direction.LEFT);
        robot.turnToAngle(0);
        robot.intake();

        waitMillis(1500);
        robot.conveyor(0.0);
        robot.stopShooter();

    }

    public void shootPS() {
    }
}
