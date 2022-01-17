package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import org.firstinspires.ftc.teamcode.vision.ClusterSort.Coordinate;

public class ExplosivesIK {

    double ARM1,ARM2,ENCODER_RESOLUTION;

    double angle0LeftConstraint=0;
    double angle1Constraint=0;


    Coordinate joint0Coord,joint1Coord,joint2Coord;

    public ExplosivesIK(double arm1Length, double arm2Length, double encoderResolution) {
        ARM1=arm1Length;
        ARM2=arm2Length;
        ENCODER_RESOLUTION=encoderResolution;
    }

    public int angleToEncoder(double angle, int joint) {
        switch (joint) {
            case(0): return (int)-((Math.round((ENCODER_RESOLUTION * (Math.PI - angle)) / (Math.PI * 2))));
            case(1): return (int) Math.round((ENCODER_RESOLUTION * (Math.PI + angle)) / (Math.PI * 2))-96;
            case(2):
                //TODO: actually think this through
                return (int) Math.round((ENCODER_RESOLUTION * (Math.PI + angle)) / (Math.PI * 2));
            default: return 0;
        }
    }



    public void addAngleConstraint(int joint, double constraint) {
        if(joint==0) {
            angle0LeftConstraint=constraint;
        } else if (joint==1) {
            angle1Constraint=constraint;
        }
    }

//    /*
//     https://www.ryanjuckett.com/analytic-two-bone-ik-in-2d/
//      */
//    //***************************************************************************************
///// calculate
///// Given a two bone chain located at the origin (bone1 is the parent of bone2), this
///// function will compute the bone angles needed for the end of the chain to line up
///// with a target position. If there is no valid solution, the angles will be set to
///// get as close to the target as possible.
/////
///// returns: True when a valid solution was found.
/////**************************************************************************************
//    public IKValues calculate
//            (
//                    Boolean solvePosangle1, // Solve for positive angle 2 instead of negative angle 2
//                    Coordinate joint0Pos,
//                    double length1,      // Length of bone 1. Assumed to be >= zero
//                    double length2,      // Length of bone 2. Assumed to be >= zero
//                    Coordinate target
//            )
//    {
//
//        joint0Coord=joint0Pos;
//
//        target = new Coordinate(target.getX()-joint0Pos.getX(),target.getY()-joint0Pos.getY());
//
//        System.out.println(target);
//
//        final double epsilon = 0.0001; // used to prevent division by small numbers
//
//        Boolean foundValidSolution = true;
//
//        double targetDistSqr = (target.getX()*target.getX() + target.getY()*target.getY());
//
//        h = Math.sqrt(targetDistSqr);
//
//        //Clip h
//        if(h>ARM1+ARM2) {
//            h=ARM1+ARM2;
//        }
//
//        System.out.println(h);
//
//        //===
//        // Compute a new value for angle1 along with its cosine
//        double sinangle1;
//        double cosangle1;
//
//        double cosangle1_denom = 2*length1*length2;
//        if( cosangle1_denom > epsilon )
//        {
//            cosangle1 =   (targetDistSqr - length1*length1 - length2*length2)
//                    / (cosangle1_denom);
//
//            // if our result is not in the legal cosine range, we can not find a
//            // legal solution for the target
//            if( (cosangle1 <= -1.0) || (cosangle1 >= 1.0) )
//                foundValidSolution = false;
//
//            // clamp our value into range so we can calculate the best
//            // solution when there are no valid ones
//            cosangle1 = Math.max(-1, Math.min( 1, cosangle1 ) );
//
//            // compute a new value for angle1
//            angle1 = Math.acos( cosangle1 );
//
//            // adjust for the desired bend direction
//            if( !solvePosangle1 )
//                angle1 = -angle1;
//
//            // compute the sine of our angle
//            sinangle1 = Math.sin(angle1);
//        }
//        else
//        {
//            // At least one of the bones had a zero length. This means our
//            // solvable domain is a circle around the origin with a radius
//            // equal to the sum of our bone lengths.
//            double totalLenSqr = (length1 + length2) * (length1 + length2);
//            if(    targetDistSqr < (totalLenSqr-epsilon)
//                    || targetDistSqr > (totalLenSqr+epsilon) )
//            {
//                foundValidSolution = false;
//            }
//
//            // Only the value of angle0 matters at this point. We can just
//            // set angle1 to zero.
//            angle1 = 0.0;
//            cosangle1 = 1.0;
//            sinangle1 = 0.0;
//        }
//
//        //===
//        // Compute the value of angle0 based on the sine and cosine of angle1
//        double triAdjacent = length1 + length2*cosangle1;
//        double triOpposite = length2*sinangle1;
//
//        double tanY = target.getY()*triAdjacent - target.getX()*triOpposite;
//        double tanX = target.getX()*triAdjacent + target.getY()*triOpposite;
//
//        // Note that it is safe to call Atan2(0,0) which will happen if targetX and
//        // targetY are zero
//        angle0 = Math.atan2( tanY, tanX );
//
//        System.out.println(angle0);
//
//        if(angle0 <=angle0Constraint) {
//            angle0 =Math.PI;
////            JOINT0_COLOR = Color.orange;
//            foundValidSolution = false;
//        }
//
//        if (angle1 <=-angle1Constraint) {
//            angle1 = -angle1Constraint;
////            JOINT1_COLOR = Color.orange;
//            foundValidSolution = false;
//        }
//
////        System.out.println(angle1);
//
//        //Calculate theta4
//        Coordinate centeredTarget = target;
////        Coordinate centeredTarget = new Coordinate(target.getX()- joint0Coord.getX(),target.getY()- joint0Coord.getY());
//
//        double theta4 = Math.atan(centeredTarget.getY()/centeredTarget.getX());
//
//        //Correct theta4
//        if(theta4<0) {
//            theta4=Math.PI+theta4;
//        }
//
//        double theta1 = angle0;
//        double theta2 = angle1;
//        double theta3 = theta4-theta1;
//
//        joint1Coord = new Coordinate(Math.cos(theta1)*ARM1,Math.sin(theta1)*ARM1);
//
//        if(angle0==Math.PI) {
////            joint2Coord = new Coordinate(Math.cos(Math.PI+angle0)*ARM2+joint1Coord.getX(),Math.sin(Math.PI+angle0)*ARM2+joint1Coord.getY());
//            angle1 = Math.atan2(centeredTarget.getY()-joint1Coord.getY(),centeredTarget.getX()-joint1Coord.getX())-Math.PI;
//            joint2Coord = new Coordinate(Math.cos(angle1+Math.PI)*ARM2+joint1Coord.getX(),Math.sin(angle1+Math.PI)*ARM2+joint1Coord.getY());
////            joint2Coord = new Coordinate(joint2Coord.getX()+joint0Coord.getX(),joint2Coord.getY()+joint0Coord.getY());
////
////            joint1Coord = new Coordinate(joint1Coord.getX()+joint0Coord.getX(),joint1Coord.getY()+joint0Coord.getY());
//        } else {
//            double finalAngle = Math.PI+angle1-(Math.PI-angle0);
//            joint2Coord = new Coordinate(Math.cos(finalAngle) * ARM2, Math.sin(finalAngle) * ARM2);
////            joint1Coord = new Coordinate(joint1Coord.getX()+joint0Coord.getX(),joint1Coord.getY()+joint0Coord.getY());
////            joint2Coord = new Coordinate(joint2Coord.getX()+joint1Coord.getX(),joint2Coord.getY()+joint1Coord.getY());
//        }
//
//
//        return new IKValues(foundValidSolution,joint0Coord,joint1Coord,joint2Coord,angle0,angle1);
//    }

    private double angle0=0.0;
    private double angle1=0.0;

    public IKValues calculate
            (
                    Boolean solvePosAngle2, // Solve for positive angle 2 instead of negative angle 2
                    double length1,      // Length of bone 1. Assumed to be >= zero
                    double length2,      // Length of bone 2. Assumed to be >= zero
                    double targetX,      // Target x position for the bones to reach
                    double targetY,       // Target y position for the bones to reach
                    Coordinate joint0Pos
            )
    {


        if(targetY<0) {
            targetY=0;
        }

        if(targetX>joint0Pos.getX()+50) {
            targetX=joint0Pos.getX()+50;
        }

        targetX = targetX-joint0Pos.getX();
        targetY = targetY-joint0Pos.getY();

        final double epsilon = 0.0001; // used to prevent division by small numbers

        Boolean foundValidSolution = true;

        double targetDistSqr = (targetX*targetX + targetY*targetY);

        //===
        // Compute a new value for angle1 along with its cosine
        double sinAngle2;
        double cosAngle2;

        double cosAngle2_denom = 2*length1*length2;
        if( cosAngle2_denom > epsilon )
        {
            cosAngle2 =   (targetDistSqr - length1*length1 - length2*length2)
                    / (cosAngle2_denom);

            // if our result is not in the legal cosine range, we can not find a
            // legal solution for the target
            if( (cosAngle2 < -1.0) || (cosAngle2 > 1.0) )
                foundValidSolution = false;

            // clamp our value into range so we can calculate the best
            // solution when there are no valid ones
            cosAngle2 = Math.max(-1, Math.min( 1, cosAngle2 ) );

            // compute a new value for angle1
            angle1 = Math.acos( cosAngle2 );

            // adjust for the desired bend direction
            if( !solvePosAngle2 )
                angle1 = -angle1;

            // compute the sine of our angle
            sinAngle2 = Math.sin( angle1 );
        }
        else
        {
            // At least one of the bones had a zero length. This means our
            // solvable domain is a circle around the origin with a radius
            // equal to the sum of our bone lengths.
            double totalLenSqr = (length1 + length2) * (length1 + length2);
            if(    targetDistSqr < (totalLenSqr-epsilon)
                    || targetDistSqr > (totalLenSqr+epsilon) )
            {
                foundValidSolution = false;
            }

            // Only the value of angle0 matters at this point. We can just
            // set angle1 to zero.
            angle1    = 0.0;
            cosAngle2 = 1.0;
            sinAngle2 = 0.0;
        }

        //===
        // Compute the value of angle0 based on the sine and cosine of angle1
        double triAdjacent = length1 + length2*cosAngle2;
        double triOpposite = length2*sinAngle2;

        double tanY = targetY*triAdjacent - targetX*triOpposite;
        double tanX = targetX*triAdjacent + targetY*triOpposite;

        // Note that it is safe to call Atan2(0,0) which will happen if targetX and
        // targetY are zero
        angle0 = Math.atan2( tanY, tanX );

        if(angle0 <= angle0LeftConstraint-(Math.PI/2)) {
            angle0 =Math.PI;
//            JOINT0_COLOR = Color.orange;
            foundValidSolution = false;
        }

        if (Math.PI+angle1 <= angle1Constraint) {
            angle1 = -Math.PI+angle1Constraint;
//            JOINT1_COLOR = Color.orange;
            foundValidSolution = false;
        }

        if(solvePosAngle2) {
            angle1=Math.PI+(angle1-Math.PI);
        }

        System.out.println("Angle: " + angle1);

        Coordinate[] jointsPos = calcJointPositionAtOrigin(new Coordinate(targetX,targetY),angle0,angle1);

        return new IKValues(foundValidSolution,joint0Pos, jointsPos[1].addedTo(joint0Pos), jointsPos[2].addedTo(joint0Pos),angle0,angle1);
    }

    public Coordinate[] calcJointPositionAtOrigin(Coordinate target, double angle0, double angle1) {

        Coordinate[] coords = {new Coordinate(0,0),new Coordinate(0,0),new Coordinate(0,0)};

        double theta4 = Math.atan(target.getY()/target.getX());

        //Correct theta4
        if(theta4<0) {
            theta4=Math.PI+theta4;
        }

        double theta1 = angle0;

        coords[1] = new Coordinate(Math.cos(theta1)*ARM1,Math.sin(theta1)*ARM1);

        if(angle0>=Math.PI- angle0LeftConstraint) {
            if (!(Math.PI+angle1 <= angle1Constraint)) {
                this.angle1 = Math.atan2(target.getY()-coords[1].getY(),target.getX()-coords[1].getX())-Math.PI;
                angle1 = Math.atan2(target.getY()-coords[1].getY(),target.getX()-coords[1].getX())-Math.PI;
            } else {
                this.angle1 = -Math.PI+angle1Constraint;
                angle1 = -Math.PI+angle1Constraint;
            }
            System.out.println("HDFKKJDFJKF");

            coords[2] = new Coordinate(Math.cos(angle1+Math.PI)*ARM2+coords[1].getX(),Math.sin(angle1+Math.PI)*ARM2+coords[1].getY());
        } else {
            double finalAngle = (Math.PI+angle1)-(Math.PI-angle0);
            coords[2] = new Coordinate(Math.cos(finalAngle) * ARM2, Math.sin(finalAngle) * ARM2);
            coords[2] = new Coordinate(coords[2].getX()+coords[1].getX(),coords[2].getY()+coords[1].getY());
            System.out.println("Finalangle: " + finalAngle);
        }

        System.out.println("Coord: " + coords[2]);

        return coords;
    }

}