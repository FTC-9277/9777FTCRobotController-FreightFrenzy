package org.firstinspires.ftc.teamcode.ExplosivesUtils;

public class SensorData {

    public String getLabel() {
        return label;
    }

    public String getData() {
        return data;
    }

    private String label;
    private String data;

    public SensorData(String label, String data) {
        this.label=label;
        this.data=data;
    }

    public SensorData(String label, double data) {
        this.label=label;
        this.data=Double.toString(data);
    }

}
