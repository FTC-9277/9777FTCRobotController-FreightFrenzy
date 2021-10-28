package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.SensorData;

import java.util.ArrayList;

/*
    Displays all sensors found in the Robot class's getSensorData() in the telemetry.
 */
@TeleOp(name = "Sensor Analytics")
public class SensorDataTele extends ExplosiveTele {
    @Override
    protected void initialize() {
        // Initialize any extra sensors here
    }

    @Override
    protected void looping() {
        ArrayList<SensorData> sensors = robot.getSensorData();
        for(SensorData data : sensors) {
            telemetry.addData(data.getLabel(),data.getData());
        }
        telemetry.update();
    }
}
