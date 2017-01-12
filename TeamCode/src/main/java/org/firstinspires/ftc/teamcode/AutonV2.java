package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/4/2016.
 */

//Plan: Launch ball into goal, reload, launch second, then hit beacons and then knock the ball

@Autonomous (name = "PrimaryAutonomousRED", group = "Sensor")
public class AutonV2 extends LinearOpMode {

    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    public Servo BallG1;
    public Servo BallG2;

    //VARIABLES FOR LAUNCHER
    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;

    public boolean hasChanged = false;

    /*
    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double x;
    */
    public boolean turnCompleted = false;

    //SENSORS
    public ColorSensor colorSensor;
    public OpticalDistanceSensor bottomOD;
    public OpticalDistanceSensor frontOD;
    public OpticalDistanceSensor backOD;

    public boolean isRed;
    public boolean isBlue;

    //BUTTON PUSHER
    public Servo buttonPusher;
    public boolean buttonPress;
    public boolean buttonInit;
    public boolean push;

    public double NumberOfRevs3;

    public double colorCheckStep = 0;

    public boolean hasStarted = false;
    public boolean pushed = false;
    public void initializeRobot () {


        //DRIVE-TRAIN MOTORS
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");
        FL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);


        //LAUNCHER MOTORS
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");
        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //SENSORS
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        bottomOD = hardwareMap.opticalDistanceSensor.get("bottomOD");
        frontOD = hardwareMap.opticalDistanceSensor.get("frontOD");
        backOD = hardwareMap.opticalDistanceSensor.get("backOD");

        //BUTTON PUSHER
        buttonPusher = hardwareMap.servo.get("buttonPusher");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        buttonInit = false;
        buttonPress = false;
        push = false;

        //super.initializeRobot();
    }
    /*
    boolean isRed () throws InterruptedException {
        if(colorSensor.red() > 1 && colorSensor.red() > colorSensor.blue()){
            colorCheckStep++;
            Thread.sleep(100);
            if(colorCheckStep == 1){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    boolean isBlue () throws InterruptedException {
        if(colorSensor.blue() > 1 && colorSensor.blue() > colorSensor.red()){
            colorCheckStep++;
            Thread.sleep(100);
            if(colorCheckStep == 1){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }
````*/
    public void runOpMode() throws InterruptedException{

        initializeRobot();

        /*
        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        */

        //SEQUENCE VARIABLE
        double step = 0;

        //REVOLUTION VARIABLES
        int NumberOfRevs1 = -200;
        int NumberOfRevs2 = -1650;

        //ANGLE VARIABLES
        double Angle1 = 180;
        double Angle2 = 280;

        //composeTelemetry();
        waitForStart();
        while(opModeIsActive()){

            bottomOD.enableLed(true);

            colorSensor.enableLed(false);
            isRed = colorSensor.red() > 1 && colorSensor.red() > colorSensor.blue() ? true : false;
            isBlue = colorSensor.blue() > 1 && colorSensor.blue() > colorSensor.red() ? true : false;

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            //telemetry.addData("x", super.x);

            telemetry.update();
/*
            //ESTABLISH ROTATION
            super.x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (super.x < 0) {
                super.x = super.x + 360;
            }
*/
            BallG1.setPosition(0);
            BallG2.setPosition(1);

            //SEQUENCES

            //MOVE FORWARD
            if(step == 0){
                /*
                if(!hasStarted) {
                    forward(100);
                    hasStarted = true;
                }
                if(turnCompleted){
                    shoot = true;
                    step = step + 1;
                }
                */
                if(FL.getCurrentPosition() > NumberOfRevs1) {
                    BL.setPower(-.25);
                    BR.setPower(-.25);
                    FR.setPower(-.25);
                    FL.setPower(-.25);
                }
                else{
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }
            /*
            if(step == .5){
                super.runOpMode(30, true);
                step = step + .25;
            }
            if(step == .75){
                super.runOpMode(0, false);
                step = step + .25;
            }
            */
            //LAUNCH BALLS
            if(step == 1){
                shoot = true;
                step = step + 1;
            }
            if(step == 2){
                if(!shoot) {
                    shoot = true;
                    step=step+1;
                }
            }
            if(shoot) {
                if (!resume) {
                    if (LauncherM.getCurrentPosition() <= 400 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                    } else if (LauncherM.getCurrentPosition() <= 525 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.08);
                    } else {
                        pause = true;
                    }
                    if (pause) {
                        //INPUT RELOAD FUNCTION WHEN READY HERE
                        LauncherM.setPower(0.03);
                        Reloader.setPosition(256);
                        resume = true;
                        pause = false;
                    }
                }
                if (resume) {
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 1250 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.05);
                    } else if (LauncherM.getCurrentPosition() > 1250 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 2255 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0);
                    } else if (LauncherM.getCurrentPosition() > 2255 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.08);
                    } else {
                        LauncherM.setPower(0);
                        resume = false;
                        EncoderClicks = EncoderClicks + 2510;
                        Thread.sleep(500);
                        shoot = false;
                    }
                }
                idle();
            }
            //Move forward
            if(step == 3){
                /*
                if(!hasStarted) {
                    forward(1500);
                    hasStarted = true;
                }
                if(turnCompleted){
                    step = step + 1;
                    turnCompleted = false;
                    hasStarted = false;
                }
                */
                if(!shoot) {
                    if (FL.getCurrentPosition() > NumberOfRevs2) {
                        BL.setPower(-.5);
                        BR.setPower(-.5);
                        FR.setPower(-.5);
                        FL.setPower(-.5);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;
                    }
                }
            }

            //Strafe for time
            if(step == 4){
                Thread.sleep(500);
                if(!hasStarted) {
                    strafeLeft(1550);
                    hasStarted = true;
                }
                if(turnCompleted){
                    step = step + .5;
                    turnCompleted = false;
                    hasStarted = false;
                }
            }
            if(step == 4.5){
                Thread.sleep(500);
                NumberOfRevs3 = FL.getCurrentPosition() - 2500;
                step = step + .5;
            }
            //Turn 180
            if(step == 5){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.25);
                    BR.setPower(.25);
                    FR.setPower(.25);
                    FL.setPower(-.25);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + .5;
                }
                //super.runOpMode(Angle1, false, false);
            }
            if(step == 5.5){
                NumberOfRevs3 = FL.getCurrentPosition() + 75;
                step = step + .5;
            }
            //Strafe for time
            if(step == 6){
                stopAtLine(1);
                if(turnCompleted) {
                    step = step + .5;
                    turnCompleted = false;
                }
            }
            if(step == 6.5){
                Thread.sleep(500);
                hasStarted = false;
                step = step + .5;
            }
            //Move to line
            if(step == 7){
                if(!hasStarted) {
                    strafeRight(650);
                    hasStarted = true;
                }
                if(turnCompleted){
                    step = step + 1;
                    turnCompleted = false;
                    hasStarted = false;
                }
            }
            //Set revs3
            if(step == 8){
                Thread.sleep(500);
                step = step + 1;
            }

            //Position
            if(step == 9){
                if(!hasStarted) {
                    NumberOfRevs3 = FL.getCurrentPosition() - 500;
                    hasStarted = true;
                }
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.25);
                    BR.setPower(-.25);
                    FR.setPower(-.25);
                    FL.setPower(-.25);
                }
                else{
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }

            //set possible rev3
            if(step == 10){
                NumberOfRevs3 = FL.getCurrentPosition() + 300;
                step=step+1;
            }

            //Detect color
            if(step == 11){
                if(isRed){
                    //push button
                    push = true;

                    if(pushed) {
                        step = step + 1;
                    }
                }
                else if(isBlue){
                    //move forward confirm and push button
                    if(FL.getCurrentPosition() < NumberOfRevs3) {
                        BL.setPower(.1);
                        BR.setPower(.1);
                        FR.setPower(.1);
                        FL.setPower(.1);
                    }
                    else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(100);
                        push = true;
                        TimeUnit.MILLISECONDS.sleep(1500);
                        if(pushed) {
                            step = step + 1;
                        }
                    }
                }
            }

            //set next rev3
            if(step == 12){
                pushed = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 100;
                step=step+1;
            }

            //move forward
            if(step == 13){
                if(!hasStarted) {
                    NumberOfRevs3 = FL.getCurrentPosition() + 100;
                    hasStarted = true;
                }
                if(FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(.5);
                    BR.setPower(.5);
                    FR.setPower(.5);
                    FL.setPower(.5);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }

            //MOVE to line
            if(step == 14){
                hasStarted = false;
                stopAtLine(1);
                if(turnCompleted){
                    step=step+1;
                    turnCompleted = false;
                }
            }

            //set revs3
            if(step == 15){
                NumberOfRevs3 = FL.getCurrentPosition() + 100;
                step=step+1;
            }

            //position
            if(step == 16){
                if(!hasStarted) {
                    NumberOfRevs3 = FL.getCurrentPosition() - 100;
                    hasStarted = true;
                }
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else{
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }

            //set possible rev3
            if(step == 17){
                hasStarted = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 300;
                step=step+1;
            }

            //Detect color
            if(step == 18){
                if(isRed()){
                    //push button
                    push = true;
                    if(pushed) {
                        step = step + .5;
                    }
                }
                else if(isBlue()){
                    //move forward confirm and push button
                    if(FL.getCurrentPosition() < NumberOfRevs3) {
                        BL.setPower(.5);
                        BR.setPower(.5);
                        FR.setPower(.5);
                        FL.setPower(.5);
                    }
                    else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(100);
                        push = true;
                        TimeUnit.MILLISECONDS.sleep(1500);
                        if(pushed) {
                            step = step + .5;
                        }
                    }
                }
            }
            if(step == 18.5){
                pushed = false;
                TimeUnit.SECONDS.sleep(2);
                NumberOfRevs3 = FL.getCurrentPosition() - 5000;
                step = step + .5;
            }
            //TURN
            if(step == 19){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.25);
                    BR.setPower(.25);
                    FR.setPower(.25);
                    FL.setPower(-.25);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
                //super.runOpMode(Angle2, false);
            }

            //set rev3
            if(step == 20){
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() - 2000;
                step=step+1;
            }

            //move forward
            if(step == 21){
                if(!hasStarted) {
                    NumberOfRevs3 = FL.getCurrentPosition() - 2000;
                    hasStarted = true;
                }
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else{
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }
            /*

            Button Pusher

             */

            if(!buttonInit){
                buttonPusher.setPosition(.5);
                if(push){
                    buttonInit = true;
                }
            }
            else{
                buttonPusher.setPosition(.4);
                sleep(700);
                buttonPusher.setPosition(.6);
                sleep(700);
                buttonInit = false;
                pushed = true;
            }
        }
    }
    //detect color
    boolean isRed () {
        if(colorSensor.red() > 1 && colorSensor.red() > colorSensor.blue()){
            colorCheckStep++;
            sleep(100);
            if(colorCheckStep == 1){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    boolean isBlue () {
        if(colorSensor.blue() > 1 && colorSensor.blue() > colorSensor.red()){
            colorCheckStep++;
            sleep(100);
            if(colorCheckStep == 1){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }
    //move backwards
    void moveBack (double clicks) {
        double runTo = FL.getCurrentPosition() + clicks;
        if(FL.getCurrentPosition() < runTo) {
            BL.setPower(.5);
            BR.setPower(.5);
            FR.setPower(.5);
            FL.setPower(.5);
        }
        else {
            BL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
            turnCompleted = true;
            return;
        }
    }
    //stop at line
    void stopAtLine (int dir) {
        if(bottomOD.getRawLightDetected() < .05){
            FL.setPower(.25 * dir);
            BL.setPower(.25 * dir);
            FR.setPower(.25 * dir);
            BR.setPower(.25 * dir);
        }
        else{
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            turnCompleted = true;
            return;
        }
    }
    //strafe left
    void strafeLeft (int time) {
        FR.setPower(-.5);
        BR.setPower(.5);
        FL.setPower(.5);
        BL.setPower(-.5);
        sleep(time);
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        turnCompleted = true;
        return;
    }
    void strafeRight (int time) {
        FR.setPower(.5);
        BR.setPower(-.5);
        FL.setPower(-.5);
        BL.setPower(.5);
        sleep(time);
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        turnCompleted = true;
        return;
    }
    //shoot
    void shoot (){
        if(shoot) {
            if (!resume) {
                if (LauncherM.getCurrentPosition() <= 400 + (EncoderClicks - 2510)) {
                    LauncherM.setPower(1);
                } else if (LauncherM.getCurrentPosition() <= 525 + (EncoderClicks - 2510)) {
                    LauncherM.setPower(.08);
                } else {
                    pause = true;
                }
                if (pause) {
                    //INPUT RELOAD FUNCTION WHEN READY HERE
                    LauncherM.setPower(0.03);
                    Reloader.setPosition(256);
                    resume = true;
                    pause = false;
                }
            }
            if (resume) {
                if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 1250 + (EncoderClicks - 2510)) {
                    LauncherM.setPower(.05);
                } else if (LauncherM.getCurrentPosition() > 1250 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 2255 + (EncoderClicks - 2510)) {
                    LauncherM.setPower(1);
                    Reloader.setPosition(0);
                } else if (LauncherM.getCurrentPosition() > 2255 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                    LauncherM.setPower(.08);
                } else {
                    LauncherM.setPower(0);
                    resume = false;
                    EncoderClicks = EncoderClicks + 2510;
                    sleep(500);
                    shoot = false;
                    return;
                }
            }
        }
    }
    //Move Forward
    public void forward (double clicks){

        double runTo = clicks;

        if(!hasChanged) {
            runTo = FL.getCurrentPosition() - clicks;
            hasChanged = true;
        }

        if(FL.getCurrentPosition() > -clicks) {
            BL.setPower(-.5);
            BR.setPower(-.5);
            FR.setPower(-.5);
            FL.setPower(-.5);
        }
        else{
            BL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
            turnCompleted = true;
            hasChanged = false;
            return;
        }
    }
    /*
    //MOVE TO ANGLE
    void MoveToAngle (double ang){
        //turning clockwise subtracts values
        //turning counter-clockwise adds values
        if (x > ang - 10 && x < ang + 10) {
            //has reached angle therefore end loop
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            turnCompleted = true;
            return;
        } else if (x < 10 + ang) {
            //turn clockwise
            FR.setPower(-.25);
            FL.setPower(.25);
            BR.setPower(-.25);
            BL.setPower(.25);
        } else if (x > ang - 10) {
            //turn counter-clockwise
            FR.setPower(0.25);
            FL.setPower(-.25);
            BR.setPower(0.25);
            BL.setPower(-.25);
        }
    }
*/
/*
    //IMU
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("x", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}


class moveRobot extends AutonV2 {
    public void forward (double clicks){
        clicks = FL.getCurrentPosition() - clicks;
        if(FL.getCurrentPosition() > clicks) {
            BL.setPower(-.5);
            BR.setPower(-.5);
            FR.setPower(-.5);
            FL.setPower(-.5);
        }
        else{
            BL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
            return;
        }
    }*/
}
