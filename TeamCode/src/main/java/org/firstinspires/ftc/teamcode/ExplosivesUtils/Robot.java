package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.ClusterSort.Coordinate;

import java.util.ArrayList;

public class Robot {

    public DcMotorEx left, right, carouselMover, bleft, fleft, fright, bright;

    public TouchSensor touch;

    public Arm arm;

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

    public void initializeArm() {
        arm.joint0.setVelocity(-500);
        while(touch.getValue()<1) {
            //wait
        }
        arm.joint0.setVelocity(0);

        arm.joint0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.joint0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void prepareForAuto() {
        arm.joint1.setPosition(0.9);
        initializeArm();
    }

    /*
        Create all the hardware objects and set them to their respective variables. Called upon initialization of the class.
     */
    private void init() {

        initGyro();

        arm = new Arm(hardwareMap,opMode);

        // Initialize all the variables
//        left = hardwareMap.get(DcMotorEx.class, "left");
//        right = hardwareMap.get(DcMotorEx.class, "right");

        bleft = hardwareMap.get(DcMotorEx.class, "bleft");
        fleft = hardwareMap.get(DcMotorEx.class, "fleft");
        fright = hardwareMap.get(DcMotorEx.class, "fright");
        bright = hardwareMap.get(DcMotorEx.class, "bright");
        carouselMover = hardwareMap.get(DcMotorEx.class, "carousel");

        touch = hardwareMap.get(TouchSensor.class,"touch");

//        right.setDirection(DcMotorSimple.Direction.REVERSE);

        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);

        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
        Returns an ArrayList of SensorData objects that should represent every used sensor on the robot.
     */
    public ArrayList<SensorData> getSensorData() {
        ArrayList<SensorData> list = new ArrayList<>();
        list.add(new SensorData("gyro heading", getHeading()));
        list.add(new SensorData("touch", touch.getValue()));
//        list.add(new SensorData("leftside enc", leftEncoder()));
//        list.add(new SensorData("rightside enc", rightEncoder()));
        list.add(new SensorData("arm0 enc", arm.joint0.getCurrentPosition()));
        list.add(new SensorData("arm1 pos", arm.joint1.getPosition()));
//        list.add(new SensorData("x accel", imu.getAcceleration().xAccel));
//        list.add(new SensorData("y accel", imu.getAcceleration().yAccel));
//        list.add(new SensorData("z accel", imu.getAcceleration().zAccel));
//        list.add(new SensorData("pos", imu.getPosition().toString()));
//        list.add(new SensorData("velocity", imu.getVelocity().toString()));
//        list.add(new SensorData("magnetic", imu.getMagneticFieldStrength().toString()));
        return list;
    }

    public double roundToDigit(double num, int places) {
        double scale = Math.pow(10, places);
        return Math.round(num * scale) / scale;
    }

    public void drive(double speed) {

        if(speed < -1 || speed > 1) { speed = speed/Math.abs(speed); }

        left(speed);
        right(speed);
//
//        opMode.telemetry.addLine("Left: " + leftEncoder());
//        opMode.telemetry.addLine("Right: " + rightEncoder());
//        opMode.telemetry.update();

//        opMode.telemetry.addLine("P: " + right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
//        opMode.telemetry.addLine("I: " + right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
//        opMode.telemetry.addLine("D: " + right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
//        opMode.telemetry.addLine("F: " + right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
//        opMode.telemetry.update();
    }

    public void drive(double left, double right) {

//        if(speed < -1 || speed > 1) { speed = speed/Math.abs(speed); }

        left(-left);
        right(-right);
//
//        opMode.telemetry.addLine("Left: " + this.left.getVelocity());
//        opMode.telemetry.addLine("Right: " + this.right.getVelocity());
//        opMode.telemetry.update();
    }

    /**
    Turns clockwise with positive values and counterclockwise with negative.
     **/
    public void turn(double speed) {
        right(-speed);
        left(speed);
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

            left(sign*calcSpeed);
            right(-sign*calcSpeed);

            diff = getHeading()-targetAngle;
        }

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
            opMode.telemetry.addLine("GYRO WAITING...");
            opMode.telemetry.update();
        }

    }

    //TODO: UPDATE THIS
    public final int MAX_DRIVE_VELOCITY = 3000;

    public void left(double speed) {
        bleft.setVelocity(-map(MAX_DRIVE_VELOCITY,speed));
        fleft.setVelocity(-map(MAX_DRIVE_VELOCITY,speed));
    }

    public void right(double speed) {
        bright.setVelocity(-map(MAX_DRIVE_VELOCITY,speed));
        fright.setVelocity(-map(MAX_DRIVE_VELOCITY,speed));
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

    public void stop() {
        left(0.0);
        right(0.0);
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

            left(speed);
            right(speed);

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
                left((-speed * speedFuncL)+leftGyroAdd);
                right((-speed * speedFuncR)+rightGyroAdd);

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
                left(-speed * speedFuncL + leftGyroAdd);
                right(-speed * speedFuncR + rightGyroAdd);

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


            left(-speed*motorSpeedFunction((leftEncoder()-targetL),ticks));
            right(-speed*motorSpeedFunction((rightEncoder()-targetR),ticks));


            if((rightEncoder()>targetR+ALLOWED_ERROR) || (leftEncoder()<targetL-ALLOWED_ERROR)) {
                break;
            }
        }

        stop();
        autoturn(initGyro,2);
    }

    public void turnEncoders(double ticks, double speed) {
        long startTime = System.currentTimeMillis();

        double initialL = leftEncoder();
        double initialR = rightEncoder();

        double finalL = initialL-ticks;
        double finalR = initialR+ticks;

        boolean keepGoing = true;

        while(keepGoing) {

            double diffL = finalL-leftEncoder();
            double diffR = finalR-rightEncoder();

            opMode.telemetry.addLine("L: " + leftEncoder());
            opMode.telemetry.addLine("diffL: " + diffL);
            opMode.telemetry.addLine("R: " + rightEncoder());
            opMode.telemetry.addLine("diffR: " + diffR);
            opMode.telemetry.update();

            left(ENC_EQUATION(diffL)*speed);
            right(ENC_EQUATION(diffR)*speed);

            if(Math.abs(diffL)<10&&Math.abs(diffR)<10 || (System.currentTimeMillis()>startTime+5000)) {
                keepGoing=false;
            }
        }

        stop();
    }

    /// Positive ticks is right, negative is left
    public void strafe(double millis, double speed) {
        long startTime = System.currentTimeMillis();

        fright(speed);
        bright(-speed);
        fleft(-speed);
        bleft(speed);

        while(System.currentTimeMillis()<startTime+millis) {

        }

        stop();
    }

    public void strafeLeftEncoders(double ticks, double speed) {
        long startTime = System.currentTimeMillis();

        double initialL = fleft.getCurrentPosition();
        double initialR = bright.getCurrentPosition();

        double finalL = initialL+ticks;
        double finalR = initialR+ticks;

        boolean keepGoing = true;

        while(keepGoing) {

            arm.aimToPosition();

            opMode.telemetry.addLine("L: " + leftEncoder());
            opMode.telemetry.addLine("R: " + rightEncoder());
            opMode.telemetry.update();

            double diffL = fleft.getCurrentPosition()-finalL;
            double diffR = bright.getCurrentPosition()-finalR;

            fright(-speed);
            bright(speed);
            fleft(speed);
            bleft(-speed);

            if(Math.abs(diffL)<10&&Math.abs(diffR)<10 || (System.currentTimeMillis()>startTime+5000)) {
                keepGoing=false;
            }
        }

        stop();
    }

    public void strafeRightEncoders(double ticks, double speed) {
        long startTime = System.currentTimeMillis();

        double initialL = fleft.getCurrentPosition();
        double initialR = bright.getCurrentPosition();

        double finalL = initialL+ticks;
        double finalR = initialR+ticks;

        boolean keepGoing = true;

        while(keepGoing) {

            arm.aimToPosition();

            opMode.telemetry.addLine("L: " + leftEncoder());
            opMode.telemetry.addLine("R: " + rightEncoder());
            opMode.telemetry.update();

            double diffL = finalL-fleft.getCurrentPosition();
            double diffR = finalR-bright.getCurrentPosition();

            fright(speed);
            bright(-speed);
            fleft(-speed);
            bleft(speed);

            if(Math.abs(diffL)<10&&Math.abs(diffR)<10 || (System.currentTimeMillis()>startTime+5000)) {
                keepGoing=false;
            }
        }

        stop();
    }

    public void driveEncoders(double ticks, double speed) {
        long startTime = System.currentTimeMillis();

        double initialL = leftEncoder();
        double initialR = rightEncoder();

        double finalL = initialL+ticks;
        double finalR = initialR+ticks;

        double initialHeading = getHeading();

        boolean keepGoing = true;

        lastMeasurementTime = System.currentTimeMillis();

        while(keepGoing) {
            arm.aimToPosition();

            double diffL = finalL-leftEncoder();
            double diffR = finalR-rightEncoder();

            double gyroAdd = 0.0;

            if(Math.abs(initialHeading-getHeading())>2.0) {
                gyroAdd = (initialHeading-getHeading()) / 75;
            }

            left(ENC_EQUATION(diffL)*speed - gyroAdd);
            right(ENC_EQUATION(diffR)*speed + gyroAdd);

            if(Math.abs(diffL)<10&&Math.abs(diffR)<10 || (System.currentTimeMillis()>startTime+5000)) {
                keepGoing=false;
            }

            updatePosition();
        }

        stop();
    }

    // Returns whether the loop should continue
    public boolean driveEncodersLoop(int ticks, int initialL, int initialR, double initialHeading, long startTime, double speed) {
        double finalL = initialL+ticks;
        double finalR = initialR+ticks;

        double diffL = finalL-leftEncoder();
        double diffR = finalR-rightEncoder();

        double gyroAdd = 0.0;

        if(Math.abs(initialHeading-getHeading())>2.0) {
            gyroAdd = (initialHeading-getHeading()) / 100;
        }

        left(ENC_EQUATION(diffL)*speed - gyroAdd);
        right(ENC_EQUATION(diffR)*speed + gyroAdd);

        updatePosition();

        if(Math.abs(diffL)<10&&Math.abs(diffR)<10 || (System.currentTimeMillis()>startTime+5000)) {
            return false;
        }

        return true;
    }

    //in millimeters
    private final int MECHANUM_WHEEL_DIAMETER = 100;
    private final double MECHANUM_WHEEL_ENCODER_RESOLUTION = 537.7;

    /*
    Returns the linear distance travelled by the encoder ticks provided in meters
     */
    public double encoderToDistance(double encoder) {
        //encoder to rotations
        double rots = encoder/MECHANUM_WHEEL_ENCODER_RESOLUTION;

        //rots to rads
        double rads = rots*2*Math.PI;

        //rads to distance (mm)
        double dist = rads*(MECHANUM_WHEEL_DIAMETER/2);

        return dist*1000;
    }

    public double x = 0.0;
    public double y = 0.0;

    public long lastMeasurementTime = System.currentTimeMillis();

    public void updatePosition() {
        double forwardDist;

        //in seconds
        double timeElapsed = (System.currentTimeMillis()-lastMeasurementTime);
        opMode.telemetry.addLine("last: " + lastMeasurementTime);
        opMode.telemetry.addLine("now: " + System.currentTimeMillis());
        forwardDist = encoderToDistance(totalVelocity())*timeElapsed;

        lastMeasurementTime = System.currentTimeMillis();

        y-=(forwardDist*Math.cos((getHeading()*Math.PI)/180))/10000000;
        x+=(forwardDist*Math.sin((getHeading()*Math.PI)/180))/10000000;

        opMode.telemetry.addLine("x: " + x);
        opMode.telemetry.addLine("y: " + y);
        opMode.telemetry.addLine("forwardDist: " + forwardDist);
        opMode.telemetry.addLine("totalVelocity: " + totalVelocity());
        opMode.telemetry.addLine("timeElapsed: " + timeElapsed);
        opMode.telemetry.update();
    }

    public void driveToPosition(double x, double y, double speed) {
        //find angle to position
        double rad = Math.atan((this.y-y)/(this.x-x));
        double heading = getHeading();
        double radWithHeading = rad-heading;
        double magnitude = Math.sqrt((x*x)+(y*y));

        opMode.telemetry.addLine("Angle to final (ignoring heading): "+rad);
        opMode.telemetry.addLine("Angle to final (with heading): "+radWithHeading);
        opMode.telemetry.addLine("Magnitude: "+magnitude);
        opMode.telemetry.addLine("RobotX: " + this.x + ", RobotY: " + this.y);
        opMode.telemetry.addLine("PosX: " + x + ", PosY: " + y);
        opMode.telemetry.update();

        waitMillis(5000);

        long startTime = System.currentTimeMillis();

        double initialBleft = bleft.getCurrentPosition();
        double initialFleft = fleft.getCurrentPosition();
        double initialBright = bright.getCurrentPosition();
        double initialFright = fright.getCurrentPosition();

        double initialHeading = getHeading();

        //fright and bleft
        double firstPower = -Math.sin(radWithHeading-(0.25*Math.PI));
        //fleft and bright
        double secondPower = -Math.sin(radWithHeading+(0.25*Math.PI));

        bleft.setVelocity(MAX_DRIVE_MOTOR_VELOCITY*speed*firstPower);
        bright.setVelocity(MAX_DRIVE_MOTOR_VELOCITY*speed*secondPower);
        fright.setVelocity(MAX_DRIVE_MOTOR_VELOCITY*speed*firstPower);
        fleft.setVelocity(MAX_DRIVE_MOTOR_VELOCITY*speed*secondPower);

        waitMillis(1000);


        //Calculate position

        int bleftTicks = bleft.getCurrentPosition();
        int fleftTicks = fleft.getCurrentPosition();
        int brightTicks = bright.getCurrentPosition();
        int frightTicks = fright.getCurrentPosition();

        boolean keepGoing = true;
        while(keepGoing) {
            int changeInBleft = bleft.getCurrentPosition()-bleftTicks;
            int changeInFleft = fleft.getCurrentPosition()-fleftTicks;
            int changeInBright = bright.getCurrentPosition()-brightTicks;
            int changeInFright = fright.getCurrentPosition()-frightTicks;

            bleftTicks = bleft.getCurrentPosition();
            fleftTicks = fleft.getCurrentPosition();
            brightTicks = bright.getCurrentPosition();
            frightTicks = fright.getCurrentPosition();

            //If going forward change is positive

            double encoderDistance = 0.0;

            encoderDistance += changeInBleft/Math.sqrt(2);
            encoderDistance += changeInFleft/Math.sqrt(2);
            encoderDistance += changeInBright/Math.sqrt(2);
            encoderDistance += changeInFright/Math.sqrt(2);

            opMode.telemetry.addLine("Change in BLEFT: " + changeInBleft);
            opMode.telemetry.addLine("Change in FLEFT: " + changeInFleft);
            opMode.telemetry.addLine("Change in BRIGHT: " + changeInBright);
            opMode.telemetry.addLine("Change in FRIGHT: " + changeInFright);
            opMode.telemetry.addLine("Distance: " + encoderDistance);
            opMode.telemetry.addLine("Distance FUNCTION: " + encoderToDistance(encoderDistance/4));
            opMode.telemetry.update();

            if(encoderToDistance(encoderDistance/4)<magnitude) {
                keepGoing=false;
            }

        }

        stop();

    }

    public double leftVelocity() {
        return (bleft.getVelocity()+fleft.getVelocity())/2;
    }

    public double rightVelocity() {
        return (bright.getVelocity()+fright.getVelocity())/2;
    }

    public double totalVelocity() {
        return(leftVelocity()+rightVelocity())/2;
    }

    //Raising this coefficient will make the robot slow down more when the encoders reach their target & vice versa
    public int SLOW_COEFFICIENT = 400;
    public double ENC_EQUATION(double val) {
        int sign = 1;
        if(val<0) {
            sign=-1;
        }
        double calc = Math.sqrt(Math.abs(val)/SLOW_COEFFICIENT)+0.1;
        if(calc>1) {
            calc=1;
        }
        return calc*sign;
    }

    public void resetEncoders() {
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double leftEncoder() {
        return -(fleft.getCurrentPosition()+bleft.getCurrentPosition())/2;
    }

    public double rightEncoder() {
        return -(fright.getCurrentPosition()+bright.getCurrentPosition())/2;
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

//        opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target), null);
//        opMode.telemetry.update();

        double speed = 0.0;

        for (int i = 0; i < levels; i++) {
            while (System.currentTimeMillis() < startTime + TURN_TIMEOUT && Math.abs(diff) > MAX_TURN_DIFF) {
//                opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target), null);
//                opMode.telemetry.addLine("speed: " + speed);
//                opMode.telemetry.update();
                speed = motorPowerFunction(diff);
                turn(speed);
                diff = calcdiff(getHeading(), target);

                arm.aimToPosition();
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
            return (heading+(360-target));
//            return -(heading+(target));
        }

        return -(target-heading);
    }

    final double RIGHT_PS=21;
    final double MID_PS=27;
    final double LEFT_PS=31.5;

    // Custom function found on Desmos to control the power of the motors based off of the given angle
    public double motorPowerFunction(double angle) {
        if(angle>0) {
            return (Math.sqrt(angle/100)/*+0.1*/)*0.1;
        } else {
            return (-Math.sqrt(-angle/100)/*-0.1*/)*0.1;
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

//    public Coordinate getPosition() {
//        double left = leftEncoder();
//        double right = rightEncoder();
//
//        double distance = (left+right)/2;
//
////        return new Coordinate((left+right)/2);
//
//    }

    public double map(double targetUp, double value) {
        return targetUp*value;
    }

    int MAX_DRIVE_MOTOR_VELOCITY = 4000;
    public void bright(double pow) {
        bright.setVelocity(map(MAX_DRIVE_MOTOR_VELOCITY,pow));
    }
    public void fright(double pow) {
        fright.setVelocity(map(MAX_DRIVE_MOTOR_VELOCITY,pow));
    }
    public void bleft(double pow) {
        bleft.setVelocity(map(MAX_DRIVE_MOTOR_VELOCITY,pow));
    }
    public void fleft(double pow) {
        fleft.setVelocity(map(MAX_DRIVE_MOTOR_VELOCITY,pow));
    }

}
