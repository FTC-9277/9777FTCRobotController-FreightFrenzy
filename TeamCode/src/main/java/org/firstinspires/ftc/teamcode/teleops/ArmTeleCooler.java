package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosivesIK;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.IKValues;
import org.firstinspires.ftc.teamcode.vision.ClusterSort.Coordinate;

//@TeleOp(name = "ArmTele but Cooler")
public class ArmTeleCooler extends ExplosiveTele {

    private static final double ARM1 = 31.2;
    private static final double ARM2 = 24;
    public static final double ENCODER_RESOLUTION = 537.7;

    public static final Coordinate JOINT0_INITIAL_POS = new Coordinate(0,0);
    public Coordinate target = new Coordinate(20,20);

    ExplosivesIK ikSolver = new ExplosivesIK(ARM1,ARM2,ENCODER_RESOLUTION);
    IKValues ik = new IKValues(false, new Coordinate(0,0),new Coordinate(0,0),new Coordinate(0,0),0.0,0.0);

    @Override
    protected void initialize() {
        ik = ikSolver.calculate(false,ARM1,ARM2,10.0,10.0,JOINT0_INITIAL_POS);
    }

    Boolean locked = false;

    double armMultiplier=1/3;

    @Override
    protected void looping() {

//        ik = ikSolver.calculate(false,ARM1,ARM2,target.getX(),target.getY(),JOINT0_INITIAL_POS);
//
//        robot.arm.PIDLoop(robot.arm.joint0,ikSolver.angleToEncoder(ik.angle0,0));
//        robot.arm.PIDLoop(robot.arm.joint1,ikSolver.angleToEncoder(ik.angle1,1));
//
//        if(gamepad1.b) {
//            armMultiplier=1/4.0;
//        } else {
//            armMultiplier=1/2.0;
//        }
//
//        if(Math.abs(gamepad1.left_stick_x)>0.2) {
//            target = target.addedTo(new Coordinate(gamepad1.left_stick_x*armMultiplier,0));
//        }
//        if(Math.abs(gamepad1.left_stick_y)>0.2) {
//            target = target.addedTo(new Coordinate(0,-gamepad1.left_stick_y*armMultiplier));
//        }
//
//
////        telemetry.addLine("-------Joint 0------");
////        telemetry.addLine("Velocity: " + robot.arm.joint0.getVelocity());
////        telemetry.addLine("Current: " + robot.arm.joint0.getCurrentPosition());
////        telemetry.addLine("Target: " + ikSolver.angleToEncoder(ik.angle0,0));
////
////        telemetry.addLine("-------Joint 1------");
////        telemetry.addLine("Velocity: " + robot.arm.joint1.getVelocity());
////        telemetry.addLine("Current: " + robot.arm.joint1.getCurrentPosition());
////        telemetry.addLine("Target: " + ikSolver.angleToEncoder(ik.angle1,1));
////
////        telemetry.addLine("-------IK-------");
////        telemetry.addLine("Target coord: " + target.toString());
////        telemetry.addLine("Joint 0 coord: " + ik.joint0Pos.toString());
////        telemetry.addLine("Joint 1 coord: " + ik.joint1Pos.toString());
////        telemetry.addLine("Joint 2 coord: " + ik.joint2Pos.toString());
////        telemetry.addLine("Joint 0 angle: " + ik.angle0);
////        telemetry.addLine("Joint 0 enc: " + ikSolver.angleToEncoder(ik.angle0,0));
////        telemetry.addLine("Joint 1 angle: " + ik.angle1);
////        telemetry.addLine("Joint 1 enc: " + ikSolver.angleToEncoder(ik.angle1,1));
//
//        telemetry.update();

    }
}
