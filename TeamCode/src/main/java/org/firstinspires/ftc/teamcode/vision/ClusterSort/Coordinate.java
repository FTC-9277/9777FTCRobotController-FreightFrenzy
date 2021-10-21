package org.firstinspires.ftc.teamcode.vision.ClusterSort;

public class Coordinate {

    private final int PLACES_TO_ROUND=2;

    private double x,y;

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public void setX(double x) {
        this.x=x;
    }

    public void setY(double y) {
        this.y=y;
    }

    public Coordinate(double x, double y) {
        this.x=x;
        this.y=y;
    }

    public String toString() {
        return "(" +round(x,PLACES_TO_ROUND) + "," + round(y,PLACES_TO_ROUND) + ")";
    }

    public static double[] getXValueOfCoordinates(Coordinate[] coords) {

        if(coords==null) {
            return null;
        }

        double[] nums = new double[coords.length];

        for(int i=0; i<coords.length;i++) {
            nums[i]=coords[i].x();
        }

        return nums;

    }

    public static double[] getYValueOfCoordinates(Coordinate[] coords) {

        if(coords==null) {
            return null;
        }

        double[] nums = new double[coords.length];

        for(int i=0; i<coords.length;i++) {
            nums[i]=coords[i].y();
        }

        return nums;

    }

    public static double distanceBetween(Coordinate coord1, Coordinate coord2) {
        return Math.sqrt(Math.pow((coord1.x()-coord2.x()),2)+Math.pow((coord1.y()-coord2.y()),2));
    }

    private static double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}
