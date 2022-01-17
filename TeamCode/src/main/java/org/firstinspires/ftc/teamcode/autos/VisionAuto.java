package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "Vision Auto")
public class VisionAuto extends LinearOpMode {

    Camera camera;
    Sampler sample;

    protected void initialize() {
        camera = new Camera(hardwareMap,false);

        sample = new Sampler(camera);
        String sampled = this.sample.sample();

        telemetry.addLine(sampled);
        telemetry.update();

        waitMillis(10000);
        waitForStart();

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

}
