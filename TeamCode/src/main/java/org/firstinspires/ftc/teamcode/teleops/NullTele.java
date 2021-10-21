package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;

import java.io.IOError;

@TeleOp(name = "Null TeleOp")
public class NullTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    // Just a number to display on the telemetry to show somethings happening
    int count = 0;

    @Override
    protected void looping() {
        count++;
        telemetry.addData("Looping!",count);
        telemetry.update();
    }

}
