package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosivesIK;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.IKValues;
import org.firstinspires.ftc.teamcode.vision.ClusterSort.Coordinate;

@TeleOp(name = "FD - FullTele - FD")
public class FullTeleFD extends ExplosiveTele {

    private static final double ARM1 = 31.2;
    private static final double ARM2 = 24;
    public static final double ENCODER_RESOLUTION = 537.7;

    public static final Coordinate JOINT0_INITIAL_POS = new Coordinate(0,22-4);
    public Coordinate target = new Coordinate(14,39-4);

    ExplosivesIK ikSolver = new ExplosivesIK(ARM1,ARM2,ENCODER_RESOLUTION);
    IKValues ik = new IKValues(false, new Coordinate(0,0),new Coordinate(0,0),new Coordinate(0,0),0.0,0.0);


    @Override
    protected void initialize() {
//        robot.arm.closeGrabber();
//        robot.arm.joint2.setPosition(0.0);
//        ik = ikSolver.calculate(false,ARM1,ARM2,0.0,0.0,JOINT0_INITIAL_POS);
    }

    public final double DAMPING = 0.5;
    double armMultiplier=1/3.0;

    double joint0Enc = -20;
    double joint1Enc = 30;

    @Override
    protected void looping() {
//        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
//            robot.drive((gamepad1.left_stick_y-gamepad1.right_stick_x)*DAMPING,(gamepad1.left_stick_y+gamepad1.right_stick_x)*DAMPING);
//        } else {
//            robot.drive(0);
//        }
//
//        if(gamepad2.left_bumper||gamepad2.right_bumper) {
//            robot.carouselMover.setPower(-0.25);
//        } else {
//            robot.carouselMover.setPower(0);
//        }
//
//        if(gamepad2.a) {
//            robot.arm.closeGrabber();
//        } else if (gamepad2.y) {
//            robot.arm.openGrabber();
//        }
//
//        if(gamepad2.dpad_up) {
//            robot.arm.joint2.setPosition(robot.arm.joint2.getPosition()+0.005);
//        } else if (gamepad2.dpad_down && robot.arm.joint2.getPosition()>0.56) {
//            robot.arm.joint2.setPosition(robot.arm.joint2.getPosition()-0.005);
//        }
//        if(robot.arm.joint2.getPosition()<0.56) {
//            robot.arm.joint2.setPosition(0.56);
//        }
//
//
//        ik = ikSolver.calculate(false,ARM1,ARM2,target.getX(),target.getY(),JOINT0_INITIAL_POS);
//
////        robot.arm.PIDLoop(robot.arm.joint0,ikSolver.angleToEncoder(ik.angle0,0));
//        robot.arm.PIDLoop(robot.arm.joint0,joint0Enc);
////        robot.arm.PIDLoop(robot.arm.joint1,ikSolver.angleToEncoder(ik.angle1,1));
//        robot.arm.PIDLoop(robot.arm.joint1,joint1Enc);
//
//        if(gamepad2.b) {
//            armMultiplier=1/2.0;
//        } else {
//            armMultiplier=2.0;
//        }
//
//        if(Math.abs(gamepad2.left_stick_y)>0.2) {
////            target = target.addedTo(new Coordinate(-gamepad2.left_stick_y*armMultiplier,0));
//            joint0Enc += gamepad2.left_stick_y*armMultiplier;
//        }
//        if(Math.abs(gamepad2.right_stick_y)>0.2) {
////            target = target.addedTo(new Coordinate(0,-gamepad2.right_stick_y*armMultiplier));
//            joint1Enc -= gamepad2.right_stick_y*armMultiplier;
//        }
//
//
//        telemetry.addLine("-------Joint 0------");
//        telemetry.addLine("Velocity: " + robot.arm.joint0.getVelocity());
////        telemetry.addLine("Velocity: " + gamepad2.left_stick_x);
////        telemetry.addLine("Velocity: " + gamepad2.left_stick_y);
//        telemetry.addLine("Current: " + robot.arm.joint0.getCurrentPosition());
////        telemetry.addLine("Target: " + joint0Enc);
//        telemetry.addLine("Target: " + ikSolver.angleToEncoder(ik.angle0,0));
////
//        telemetry.addLine("-------Joint 1------");
//        telemetry.addLine("Velocity: " + robot.arm.joint1.getVelocity());
//        telemetry.addLine("Current: " + robot.arm.joint1.getCurrentPosition());
////        telemetry.addLine("Target: " + joint1Enc);
//        telemetry.addLine("Target: " + ikSolver.angleToEncoder(ik.angle1,1));
////
//        telemetry.addLine("-------IK-------");
//        telemetry.addLine("Target coord: " + target.toString());
//        telemetry.addLine("Joint 0 coord: " + ik.joint0Pos.toString());
//        telemetry.addLine("Joint 1 coord: " + ik.joint1Pos.toString());
//        telemetry.addLine("Joint 2 coord: " + ik.joint2Pos.toString());
//        telemetry.addLine("Joint 0 angle: " + ik.angle0);
//        telemetry.addLine("Joint 0 enc: " + ikSolver.angleToEncoder(ik.angle0,0));
//        telemetry.addLine("Joint 1 angle: " + ik.angle1);
//        telemetry.addLine("Joint 1 enc: " + ikSolver.angleToEncoder(ik.angle1,1));
//
//        telemetry.addLine("P: "+robot.arm.joint0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).p);
//        telemetry.addLine("I: "+robot.arm.joint0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).i);
//        telemetry.addLine("D: "+robot.arm.joint0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).d);
//        telemetry.addLine("F: "+robot.arm.joint0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).f);
//
//        telemetry.update();


    }
}
