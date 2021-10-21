package org.firstinspires.ftc.teamcode.ExplosivesUtils;

public class SensorData {

    public String getLabel() {
        return label;
    }

    public double getData() {
        return data;
    }

    private String label;
    private double data;

    public SensorData(String label, double data) {
        this.label=label;
        this.data=data;
    }

}
