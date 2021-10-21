package org.firstinspires.ftc.teamcode.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "Vision")
public class VisionAuto extends ExplosiveAuto {

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
    protected void begin() {
        log("cycling");
        camera.cycle();
        log("end");
        log("sampling");
        double[] value = sampler.sample();
        log(("Sample: " + value[0] + ", TotalW: " + value[1] + ", HeightUp: " + value[2] + ", HeightDown: " + value[3] + ", W: " + value[4] + ", H: " + value[5] + ", center: (" + value[6] + "," + value[7] + ")"));
        waitMillis(10000);
    }
}
