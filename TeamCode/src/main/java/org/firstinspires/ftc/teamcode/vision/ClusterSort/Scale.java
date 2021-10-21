package org.firstinspires.ftc.teamcode.vision.ClusterSort;

public class Scale {

    private Coordinate min = new Coordinate(0,0);
    private Coordinate max = new Coordinate(0,0);

    public Scale(Coordinate[] coords) {
        if(coords.length==0) {return;}
        if(coords.length==1) { min=coords[0];max=coords[0]; }

        //Find min and max for x and y

        //minX
        double minX=Double.MAX_VALUE;
        for(Coordinate coord : coords) {
            if(coord.x()<minX) { minX = coord.x(); }
        }

        //minY
        double minY=Double.MAX_VALUE;
        for(Coordinate coord : coords) {
            if(coord.y()<minY) { minY = coord.y(); }
        }

        //maxX
        double maxX=Double.MIN_VALUE;
        for(Coordinate coord : coords) {
            if(coord.x()>maxX) { maxX = coord.x(); }
        }

        //maxY
        double maxY=Double.MIN_VALUE;
        for(Coordinate coord : coords) {
            if(coord.y()>maxY) { maxY = coord.y(); }
        }

        min = new Coordinate(minX,minY);
        max = new Coordinate(maxX,maxY);

    }

    public String toString() {
        return "Scale<Min: " + min + ", Max: " + max + ">";
    }

    /**
     *
     * @return The Coordinate of the center of the scale
     */
    public Coordinate getCenter() {
        Coordinate center = new Coordinate(((max.x())+(min.x()))/2,((max.y())+(min.y()))/2);
        return center;
    }

    public double getWidth() {
        return Math.abs(min.x())+max.x();
    }

    public double getHeight() {
        return Math.abs(min.y())+max.y();
    }

}
