package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import org.firstinspires.ftc.teamcode.vision.ClusterSort.Coordinate;

public class IKValues {

    public double angle0,angle1;

    public Coordinate joint0Pos,joint1Pos,joint2Pos;

    public Boolean foundValidSolution = false;

    public IKValues(Boolean foundValidSolution, Coordinate joint0Pos, Coordinate joint1Pos, Coordinate joint2Pos, double angle0, double angle1) {
        this.angle0=angle0;
        this.angle1=angle1;
        this.joint0Pos=joint0Pos;
        this.joint1Pos=joint1Pos;
        this.joint2Pos=joint2Pos;
        this.foundValidSolution=foundValidSolution;
    }

}
