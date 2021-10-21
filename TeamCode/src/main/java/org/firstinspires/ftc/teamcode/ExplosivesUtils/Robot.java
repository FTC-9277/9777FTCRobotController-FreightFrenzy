package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

public class Robot {

    public DcMotor intake, conveyor;
    public DcMotorEx shootL, shootR, fleft, fright, bright, bleft;
    public ModernRoboticsI2cRangeSensor sonic;
    public Servo wobbler, grabber;

    BNO055IMU imu;

    OpMode opMode;

    HardwareMap hardwareMap;

    public enum Direction {
        LEFT, RIGHT
    }

    public Robot(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap=hardwareMap;
        this.opMode=opMode;
        init();
    }

    /*
        Create all the hardware objects and set them to their respective variables. Called upon initialization of the class.
     */
    private void init() {

        initGyro();

        // Initialize all the variables
        fleft = hardwareMap.get(DcMotorEx.class, "fleft");
        fright = hardwareMap.get(DcMotorEx.class, "fright");
        bleft = hardwareMap.get(DcMotorEx.class, "bleft");
        bright = hardwareMap.get(DcMotorEx.class, "bright");

        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");

        wobbler = hardwareMap.get(Servo.class, "wobbler");
        grabber = hardwareMap.get(Servo.class, "grabber");

        sonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"sonic");

        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLPID = new PIDController(0.35,0,0);
        shootRPID = new PIDController(0.35,0,0);

        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        PIDFCoefficients coefficients = new PIDFCoefficients(0.1,0.001,0.0001,0);
//
//        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);
//        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);

//        wobbler.setPosition(0.0);
        closeGrabber();
        liftWobbler();

    }

    /*
        Returns an ArrayList of SensorData objects that should represent every used sensor on the robot.
     */
    public ArrayList<SensorData> getSensorData() {
        ArrayList<SensorData> list = new ArrayList<>();
        list.add(new SensorData("gyro heading", getHeading()));
        list.add(new SensorData("fleft encoder", fleft.getCurrentPosition()));
        list.add(new SensorData("fright encoder", fright.getCurrentPosition()));
        list.add(new SensorData("bleft encoder", bleft.getCurrentPosition()));
        list.add(new SensorData("bright encoder", bright.getCurrentPosition()));
        list.add(new SensorData("leftside enc", leftEncoder()));
        list.add(new SensorData("rightside end", rightEncoder()));
        list.add(new SensorData("shootL encoder", shootL.getCurrentPosition()));
        list.add(new SensorData("shootR encoder", shootR.getCurrentPosition()));
        list.add(new SensorData("sonic", sonic.getDistance(DistanceUnit.INCH)));
        return list;
    }

    long lastShooterCheckTime=0;
    double lastShootLEncoder, lastShootREncoder;

    PIDController shootLPID, shootRPID;

    private void shoot(int ticksPerSecond) {
        shootL.setVelocity(-ticksPerSecond);
        shootR.setVelocity(-ticksPerSecond);
    }

    public void shoot(ShooterSpeed speed) {
        opMode.telemetry.addLine("ShootL: " + shootL.getVelocity());
        opMode.telemetry.addLine("ShootR: " + shootR.getVelocity());
        opMode.telemetry.update();
        switch (speed) {
            case FULL_FORWARD:
                shootL.setVelocity(-2000);
                shootR.setVelocity(-2000);
                break;
            case FULL_BACKWARD:
                shootL.setVelocity(2000);
                shootR.setVelocity(2000);
                break;
            case AUTO_PS:
                //Original -1950, -1850
                shootL.setVelocity(-1750);
                shootR.setVelocity(-1650);
                break;
            case TELEOP:
                //Original -1950, -1850
                shootL.setVelocity(-1820);
                shootR.setVelocity(-1720);
                break;
            case STOP:
                shootL.setVelocity(0);
                shootR.setVelocity(0);
                break;
            case AUTO_TOWERGOAL:
                shootL.setVelocity(-850);
                shootR.setVelocity(-850);
                break;
            case AUTO_PS_HIGH:
                shootL.setVelocity(-1850);
                shootL.setVelocity(-1750);
                break;
        }
    }

    public void stopShooter() {
        shootL.setVelocity(0);
        shootR.setVelocity(0);
    }

    public enum ShooterSpeed {
        FULL_FORWARD,AUTO_PS,FULL_BACKWARD,STOP,AUTO_TOWERGOAL, TELEOP,AUTO_PS_HIGH;
    }

    public double roundToDigit(double num, int places) {
        double scale = Math.pow(10, places);
        return Math.round(num * scale) / scale;
    }

    public void intake() {
        intake.setPower(1.0);
    }

    public void outtake() {
        intake.setPower(-1.0);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void stop() {
        drive(0);
    }

    public void conveyor(double speed) {
        conveyor.setPower(speed);
    }

    public void openGrabber() {
        grabber.setPosition(0.7);
    }

    public void closeGrabber() {
        grabber.setPosition(0.977);
    }

    public void dropWobbler() {
        wobbler.setPosition(0.588);
    }

    public void liftWobbler() {
        wobbler.setPosition(0.028);
    }

    public void middleWobbler() {
        wobbler.setPosition(0.5);
    }

    public void satisfyingWobblerDrop() {
        while(wobbler.getPosition()<0.49) {
            middleWobbler();
        }

        openGrabber();
        waitMillis(800);
        closeGrabber();
        liftWobbler();
    }

    public void drive(double speed) {

        speed = -speed;

        if(speed < -1 || speed > 1) { return; }

        fleft(speed);
        fright(speed);
        bleft(speed);
        bright(speed);

    }

    public void turn(double speed, Direction direction) {
        speed = Math.abs(speed);
        if(direction == Direction.LEFT) {
            rightSide(speed);
            leftSide(-speed);
        } else {
            rightSide(-speed);
            leftSide(speed);
        }
    }

    public void turn(double speed) {
        rightSide(-speed);
        leftSide(speed);
    }

    public void turn(int degrees, double speed) {

        long startTime = System.currentTimeMillis();

        double initialAngle = getHeading();
        double targetAngle = getHeading()+degrees;

        // If the degrees is positive, sign is 1; else -1
        int sign = degrees>0 ? 1 : -1;

        double diff = getHeading()-targetAngle;
        double calcSpeed = speed;

        while(diff >= 2 || System.currentTimeMillis()-startTime>TURN_TIMEOUT) {

            // Calculate speed that each side should go at
            if(diff>25) {
                calcSpeed=speed;
            } else if (diff>15) {
                calcSpeed=speed*0.6;
            } else if (diff>5) {
                calcSpeed=speed*0.4;
            } else {
                calcSpeed=speed*0.2;
            }

            if(calcSpeed<=MIN_TURN_SPEED) {
                calcSpeed=MIN_TURN_SPEED;
            }

            leftSide(sign*calcSpeed);
            rightSide(-sign*calcSpeed);

            diff = getHeading()-targetAngle;
        }

    }

    public double getSonic() {
        return sonic.getDistance(DistanceUnit.INCH);
    }

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {
            //Wait
        }

    }

    public void leftSide(double speed) {
        bleft(speed);
        fleft(speed);
    }

    public void rightSide(double speed) {
        bright(speed);
        fright(speed);
    }

    public Orientation getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getHeading() {
        return getAngle().firstAngle;
    }

    public void newDriveStraight(double speed, int ticks) {
//        resetEncoders();

        drive(0.5);

        int count=0;

        while(leftEncoder()<ticks && rightEncoder()<ticks) {
            opMode.telemetry.addLine(" " + count);
            opMode.telemetry.addLine("leftEnc: " + leftEncoder());
            opMode.telemetry.addLine("rightEnc: " + rightEncoder());
            opMode.telemetry.update();
            count++;
        }

        stop();
    }

    public void driveStraight(double speed, int ticks) {
//        resetEncoders();
        speed= -speed;
        long startTime = System.currentTimeMillis();
        double leftMult = 1;
        double rightMult = 1;
        while(System.currentTimeMillis()-startTime<=15000) {

            opMode.telemetry.addLine("leftEnc: " + leftEncoder());
            opMode.telemetry.addLine("rightEnc: " + rightEncoder());
            opMode.telemetry.addLine("leftMult: " + leftMult);
            opMode.telemetry.addLine("rightMult: " + rightMult);
            opMode.telemetry.update();

            leftSide(speed);
            rightSide(speed);

            // If the difference between the two is less than 10 ticks, have them both move at the same speed
            if(Math.abs(leftEncoder()-rightEncoder())<10) {
                leftMult=1;
                rightMult=1;
            } else {

                double avg = (leftEncoder()+rightEncoder())/2;

                double leftDiff = Math.abs(avg-leftEncoder());
                double rightDiff = Math.abs(avg-rightEncoder());

                if(leftEncoder()>rightEncoder()) {
                    rightMult=1.1;
                    leftMult=0.9;
                } else {
                    leftMult=1.1;
                    rightMult=0.9;
                }

            }

            if(Math.abs(ticks-leftEncoder())<10) {
                leftMult=0;
            }

            if(Math.abs(ticks-rightEncoder())<10) {
                rightMult=0;
            }

//            leftSide(speed*leftMult);
//            rightSide(speed*rightMult);



            if(leftMult==0 && rightMult==0) {
                opMode.telemetry.addLine("DONE");
                opMode.telemetry.update();
                return;
            }

        }

        opMode.telemetry.addLine("DONE");
        opMode.telemetry.update();
    }

    final int ALLOWED_ERROR = 10;

    public void throttleDrive(double speed, int ticks) {
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();
        double initGyro = getHeading();

        if(speed>0) {

            double targetL = initL + ticks;
            double targetR = initR - ticks;

            while ((Math.abs(targetR - rightEncoder()) > ALLOWED_ERROR && Math.abs(targetL - leftEncoder()) > ALLOWED_ERROR)) {
                double diff = getHeading() - initGyro;

                double percentL = (1 - ((targetL-leftEncoder()) / ticks));
                double percentR = (1 - (Math.abs(targetR-rightEncoder()) / ticks));

                double speedFuncL = motorSpeedFunction((targetL-leftEncoder()), ticks);
                double speedFuncR = motorSpeedFunction(Math.abs(targetR-rightEncoder()), ticks);

                double leftGyroAdd = 0;
                double rightGyroAdd = 0;

                if (diff > 0) {
                    rightGyroAdd = diff / 25;
                } else {
                    leftGyroAdd = -diff / 25;
                }


//                opMode.telemetry.addLine("PercentL: " + Math.round(percentL * 100) + "%, PercentR: " + Math.round(percentR * 100) + "%");
//                opMode.telemetry.addLine("SpeedFuncL: " + Math.round(speedFuncL * 100)/100.0 + ", SpeedFuncR: " + Math.round(speedFuncR * 100)/100.0);
//                opMode.telemetry.addLine("Gyro diff: " + diff);
//                opMode.telemetry.addLine("LeftGyroAdd: " + leftGyroAdd + ", RightGyroAdd: " + rightGyroAdd);
//                opMode.telemetry.update();
                leftSide((-speed * speedFuncL)+leftGyroAdd);
                rightSide((-speed * speedFuncR)+rightGyroAdd);

                if ((rightEncoder() < targetR - ALLOWED_ERROR) || (leftEncoder() > targetL + ALLOWED_ERROR)) {
                    break;
                }
            }

        } else {

            double targetL = initL - ticks;
            double targetR = initR + ticks;

            while ((Math.abs(targetR - rightEncoder()) > ALLOWED_ERROR && Math.abs(targetL - leftEncoder()) > ALLOWED_ERROR)) {
                double diff = getHeading() - initGyro;

                double percentL = (1 - ((leftEncoder()-targetL) / ticks));
                double percentR = (1 - (Math.abs(rightEncoder()-targetR) / ticks));

                double speedFuncL = motorSpeedFunction((leftEncoder()-targetL), ticks);
                double speedFuncR = motorSpeedFunction(Math.abs(rightEncoder()-targetR), ticks);

                double leftGyroAdd = 0;
                double rightGyroAdd = 0;

                if (diff > 0) {
                    leftGyroAdd = -diff / 25;
                } else {
                    rightGyroAdd = diff / 25;
                }

                opMode.telemetry.addLine("PercentL: " + Math.round(percentL * 100) + "%, PercentR: " + Math.round(percentR * 100) + "%");
                opMode.telemetry.addLine("SpeedFuncL: " + speedFuncL);
                opMode.telemetry.addLine("SpeedFuncR: " + speedFuncR);
                opMode.telemetry.addLine("Gyro diff: " + diff);
                opMode.telemetry.addLine("LeftGyroAdd: " + leftGyroAdd + ", RightGyroAdd: " + rightGyroAdd);
                opMode.telemetry.update();
                leftSide(-speed * speedFuncL + leftGyroAdd);
                rightSide(-speed * speedFuncR + rightGyroAdd);

                if ((rightEncoder() > targetR - ALLOWED_ERROR) || (leftEncoder() < targetL + ALLOWED_ERROR)) {
                    break;
                }
            }

        }


        stop();
        autoturn(initGyro,2);


    }

    public void driveBackwardEncoders(double speed, int ticks) {
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL-ticks;
        double targetR = initR+ticks;

        double initGyro = getHeading();

        while((Math.abs(targetR-rightEncoder()) > ALLOWED_ERROR && Math.abs(targetL-leftEncoder()) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + (-speed*motorSpeedFunction((leftEncoder()-targetL),ticks)));
            opMode.telemetry.addLine("rightPow: " + (-speed*motorSpeedFunction((rightEncoder()-targetR),ticks)));
            opMode.telemetry.addLine("speedFuncL: " + motorSpeedFunction((leftEncoder()-targetL),ticks));
            opMode.telemetry.addLine("speedFuncR: " + motorSpeedFunction(Math.abs(rightEncoder()-targetR),ticks));
//            opMode.telemetry.addLine("speedFuncR: " + motorSpeedFunction(rightEncoder(),targetR,initR));
            opMode.telemetry.addLine("leftEnc:" + leftEncoder() + ", leftTarget:" + targetL + ", diff:" + (leftEncoder()-targetL));
            opMode.telemetry.addLine("rightEnc:" + rightEncoder() + ", rightTarget:" + targetR + ", diff:" + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();
//            leftSide(speed*motorSpeedFunction((targetL-leftEncoder()),ticks));//-(0.3*motorPowerFunction(getHeading()-initGyro)));
//            rightSide(speed*motorSpeedFunction((targetR-rightEncoder()),ticks));
//            rightSide((speed+motorSpeedFunction(rightEncoder(),targetR,initR)));//+(0.3*motorPowerFunction(getHeading()-initGyro)));


            leftSide(-speed*motorSpeedFunction((leftEncoder()-targetL),ticks));
            rightSide(-speed*motorSpeedFunction((rightEncoder()-targetR),ticks));


            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()<targetL-ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
        autoturn(initGyro,2);
    }

    public void strafe(double speed, Direction direction) {
        if(direction == Direction.LEFT) {
            fright(-speed);
            bright(speed);
            fleft(speed);
            bleft(-speed);
        } else {
            fright(speed);
            bright(-speed);
            fleft(-speed);
            bleft(speed);
        }
    }

    public void strafeEncoders(double speed, int ticks, Direction direction) {
        speed = Math.abs(speed);
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL+ticks;
        double targetR = initR+ticks;

        double initGyro = getHeading();

        while((Math.abs(rightEncoder()-targetR) > ALLOWED_ERROR && Math.abs(leftEncoder()-targetL) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("rightPow: " + speed+(-0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("leftTurnPow: " + 0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("rightTurnPow: " + -0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("targetL: " + targetL);
            opMode.telemetry.addLine("targetR: " + targetR);
            opMode.telemetry.addLine("leftenc: " + leftEncoder());
            opMode.telemetry.addLine("rightenc: " + rightEncoder());
            opMode.telemetry.addLine("lefterr: " + Math.abs(leftEncoder()-targetL));
            opMode.telemetry.addLine("righterr: " + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();

            fright(-speed);
            fleft(speed);
            bright(speed);
            bleft(-speed);

//            if(direction == Direction.LEFT) {
//                fright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            } else {
//                fright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            }

            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()>targetL+ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
    }

    public void throttleStrafeLeft(double speed, int ticks) {
        speed = Math.abs(speed);
        resetEncoders();
        double initL = leftEncoder();
        double initR = rightEncoder();

        double targetL = initL+ticks;
        double targetR = initR+ticks;

        double initGyro = getHeading();

        while((Math.abs(rightEncoder()-targetR) > ALLOWED_ERROR && Math.abs(leftEncoder()-targetL) > ALLOWED_ERROR)) {
            opMode.telemetry.addLine("leftPow: " + speed+(0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("rightPow: " + speed+(-0.3*motorPowerFunction(getHeading()-initGyro)));
            opMode.telemetry.addLine("leftTurnPow: " + 0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("rightTurnPow: " + -0.3*motorPowerFunction(getHeading()-initGyro));
            opMode.telemetry.addLine("targetL: " + targetL);
            opMode.telemetry.addLine("targetR: " + targetR);
            opMode.telemetry.addLine("leftenc: " + leftEncoder());
            opMode.telemetry.addLine("rightenc: " + rightEncoder());
            opMode.telemetry.addLine("lefterr: " + Math.abs(leftEncoder()-targetL));
            opMode.telemetry.addLine("righterr: " + Math.abs(rightEncoder()-targetR));
            opMode.telemetry.update();

            fright(-speed);
            fleft(speed);
            bright(speed);
            bleft(-speed);

//            if(direction == Direction.LEFT) {
//                fright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            } else {
//                fright.setPower(speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                fleft.setPower(-speed-(0*motorPowerFunction(getHeading()-initGyro)));
//                bright.setPower(-speed+(0*motorPowerFunction(getHeading()-initGyro)));
//                bleft.setPower(speed-(0*motorPowerFunction(getHeading()-initGyro)));
//            }

            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()>targetL+ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
    }

    public void resetEncoders() {
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double leftEncoder() {
        return -bleft.getCurrentPosition()+100000;
//        return (bleft.getCurrentPosition()+fleft.getCurrentPosition())/2;
    }

    public double rightEncoder() {
        return bright.getCurrentPosition()+100000;
//        return (bright.getCurrentPosition()+fright.getCurrentPosition())/2;
    }

    final double MIN_TURN_SPEED = 0.2;
    final int TURN_TIMEOUT = 4000;
    final static double MAX_TURN_DIFF = 2;

    public void turnToAngle(double angle) {
        autoturn(angle,5);
    }

    public void autoturn(double angle, int levels) {
        resetEncoders();
        long startTime = System.currentTimeMillis();

        double initial = getHeading();
        double target = angle;

        double diff = calcdiff(initial, target);

        opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target), null);
        opMode.telemetry.update();

        double speed = 0.0;

        for (int i = 0; i < levels; i++) {
            while (System.currentTimeMillis() < startTime + TURN_TIMEOUT && Math.abs(diff) > MAX_TURN_DIFF) {
                opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target), null);
                opMode.telemetry.addLine("speed: " + speed);
                opMode.telemetry.update();
                speed = motorPowerFunction(diff);
                turn(speed);
                diff = calcdiff(getHeading(), target);
            }

            turn(0);

//                waitMillis(100);

            diff = calcdiff(getHeading(), target);
        }


    }

    private double calcdiff(double heading, double target) {
        heading+=180;
        target+=180;

        if(target-heading>180) {
            return -(heading+(360-target));
        }

        return target-heading;
    }

    final double RIGHT_PS=21;
    final double MID_PS=27;
    final double LEFT_PS=31.5;

    public void turnToPS(PowerShotPos pos) {
        switch(pos) {
            case RIGHT:
                autoturn(RIGHT_PS,5);
                autoturn(RIGHT_PS,5);
                break;
            case MID:
                autoturn(MID_PS,5);
                autoturn(MID_PS,5);
                break;
            case LEFT:
                autoturn(LEFT_PS,5);
                autoturn(LEFT_PS,5);
                break;
        }
    }

    public enum PowerShotPos {
        LEFT,MID,RIGHT;
    }

    // Custom function found on Desmos to control the power of the motors based off of the given angle
    public double motorPowerFunction(double angle) {
        if(angle>0) {
            return (Math.sqrt(angle/100)/*+0.1*/)*0.7;
        } else {
            return (-Math.sqrt(-angle/100)/*-0.1*/)*0.7;
        }
    }

    // Custom function found on Desmos to control the power of the motors based off the target encoder ticks and current encoder ticks
    public double motorSpeedFunction(double currentDiff, double total) {
        double percent = 1-(currentDiff/total);
        double val = 0.2;
        if(percent==0) {
            val = 0.2;
        } else if (percent<=0.2&&percent>0) {
            val = (0.2+3*percent);
        } else if (percent>0.2){
            val = (0.98 - 0.9 * percent);
        } else if(percent<0) {
            val = (0.2);
        }
        return absClipped(val,1);
//        if(percent>=0) {
//            if(percent<0.2) {
//                return (2.5*(percent+0.1));
//            } else if(percent<0.6) {
//                return 1;
//            } else {
//                return(1-(10*(percent-0.6)));
//            }
////            return (-3 * Math.pow((percent - 0.5), 2) + 1);x
//        } else {
//            if(percent>-0.2) {
//                return (-20*percent);
//            } else if(percent>-0.6) {
//                return 1;
//            } else {
//                return(-(1-(10*(-percent-0.6))));
//            }
////            return (3 * Math.pow((-percent - 0.5), 2) + 1);
//        }
    }

    // Takes a value and returns the value clipped so that it does not exceed a given max in either direction
    public double absClipped(double val, double max) {
        if(val>max) {
            return max;
        } else if (val<-max) {
            return -max;
        } else {
            return val;
        }
    }

    public double percentThere(double current, double target, double inital) {
        return (current-inital)/(target-inital);
    }

    // Manipulates a given angle so that it is between 0º and 360º
    public double stripAngle(double angle) {
        if(angle>360) {
            angle-=Math.floor(angle/360)*360;
        } else if (angle<-360) {
            angle+=(Math.floor(angle/360)*-360)-360;
        }
        return angle;
    }

    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

    public double map(double targetUp, double value) {
        return targetUp*value;
    }

    final int MAX_DRIVE_VELOCITY=2700;

    public void fright(double val) {
        fright.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void fleft(double val) {
        fleft.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void bleft(double val) {
        bleft.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

    public void bright(double val) {
        bright.setVelocity(map(MAX_DRIVE_VELOCITY,val));
    }

}
