package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous (name = "PrimaryAutonomousBLUE", group = "Sensor")
public class AutonV3 extends LinearOpMode {

    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;


    //VARIABLES FOR LAUNCHER
    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;


    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double x;
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

        buttonInit = false;
        buttonPress = false;
        push = false;
    }

    public void runOpMode() throws InterruptedException{

        initializeRobot();


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


        //SEQUENCE VARIABLE
        int step = 0;

        //REVOLUTION VARIABLES
        int NumberOfRevs1 = -300;
        int NumberOfRevs2 = -500;

        //ANGLE VARIABLES
        double Angle1 = 190;
        double Angle2 = 280;

        composeTelemetry();
        waitForStart();
        while(opModeIsActive()){

            bottomOD.enableLed(true);

            colorSensor.enableLed(false);
            //isRed = colorSensor.red() > 1 && colorSensor.red() > colorSensor.blue() ? true : false;
            //isBlue = colorSensor.blue() > 1 && colorSensor.blue() > colorSensor.red() ? true : false;

            //isRed();
            //isBlue();

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("x", x);


            telemetry.update();

            //ESTABLISH ROTATION
            x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (x < 0) {
                x = x + 360;
            }

            //SEQUENCES

            //MOVE FORWARD
            if(step == 0){
                if(FL.getCurrentPosition() > NumberOfRevs1) {
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
                    step=step+1;
                }
            }

            //LAUNCH BALLS
            if(step == 1){
                shoot = true;
                step=step+1;
            }
            if(step == 2){
                if(!shoot) {
                    shoot = true;
                    step=step+1;
                }
            }

            //Move forward
            if(step == 3){
                if(FL.getCurrentPosition() > NumberOfRevs2) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step=step+1;
                }
            }

            //Strafe for time
            if(step == 4){
                FR.setPower(.5);
                BR.setPower(-.5);
                FL.setPower(-.5);
                BL.setPower(.5);
                Thread.sleep(1000);
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                step=step+1;
            }

            //move to line
            if(step == 5){
                if(bottomOD.getRawLightDetected() < .06){
                    FL.setPower(-.5);
                    BL.setPower(-.5);
                    FR.setPower(-.5);
                    BR.setPower(-.5);
                }
                else{
                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    step=step+1;
                }
            }

            //set revs3
            if(step == 6){
                NumberOfRevs3 = FL.getCurrentPosition() - 100;
                step=step+1;
            }

            //Position
            if(step == 7){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step=step+1;
                }
            }

            //set possible rev3
            if(step == 7){
                NumberOfRevs3 = FL.getCurrentPosition() - 100;
                step=step+1;
            }

            //Detect color
            if(step == 8){
                if(isRed()){
                    //push button
                    push = true;
                    Thread.sleep(2000);
                    step=step+1;
                }
                else if(isBlue()){
                    //move forward confirm and push button
                    if(FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.1);
                        BR.setPower(-.1);
                        FR.setPower(-.1);
                        FL.setPower(-.1);
                    }
                    else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        Thread.sleep(100);
                        push = true;
                        Thread.sleep(2000);
                        step=step+1;
                    }
                }
            }

            //set next rev3
            if(step == 9){
                NumberOfRevs3 = FL.getCurrentPosition() - 100;
                step=step+1;
            }

            //move forward
            if(step == 10){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step=step+1;
                }
            }

            //MOVE to line
            if(step == 11){
                if(bottomOD.getRawLightDetected() < .06){
                    FL.setPower(-.5);
                    BL.setPower(-.5);
                    FR.setPower(-.5);
                    BR.setPower(-.5);
                }
                else{
                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    step=step+1;
                }
            }

            //set revs3
            if(step == 12){
                NumberOfRevs3 = FL.getCurrentPosition() - 100;
                step=step+1;
            }

            //position
            if(step == 13){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(-.5);
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step=step+1;
                }
            }

            //set possible rev3
            if(step == 14){
                NumberOfRevs3 = FL.getCurrentPosition() - 100;
                step=step+1;
            }

            //Detect color
            if(step == 15){
                if(isRed()){
                    //push button
                    push = true;
                    Thread.sleep(2000);
                    step=step+1;
                }
                else if(isBlue()){
                    //move forward confirm and push button
                    if(FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.5);
                        BR.setPower(-.5);
                        FR.setPower(-.5);
                        FL.setPower(-.5);
                    }
                    else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        Thread.sleep(100);
                        push = true;
                        Thread.sleep(2000);
                        step=step+1;
                    }
                }
            }

            //TURN
            if(step == 16){
                MoveToAngle(Angle2);
                if(turnCompleted){
                    step=step+1;
                }
            }

            //set rev3
            if(step == 17){
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() - 2000;
                step=step+1;
            }

            //move forward
            if(step == 18){
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
                    step=step+1;
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
                Thread.sleep(700);
                buttonPusher.setPosition(.6);
                Thread.sleep(700);
                buttonInit = false;
            }

            /*

            SHOOTING SYSTEM

             */

            if(shoot) {
                if(!resume) {
                    if (LauncherM.getCurrentPosition() <= 400 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                    } else if (LauncherM.getCurrentPosition() <= 525 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.08);
                    }
                    else{
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
                if(resume) {
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks-2510) && LauncherM.getCurrentPosition() <= 1300 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.05);
                    }
                    else if (LauncherM.getCurrentPosition() > 1300 + (EncoderClicks-2510) && LauncherM.getCurrentPosition() <= 1800 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0);
                    } else if (LauncherM.getCurrentPosition() > 1800 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
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
    void forward (double clicks){
        double runTo = FL.getCurrentPosition() - clicks;
        if(FL.getCurrentPosition() > runTo) {
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
            return;
        }
    }

    //MOVE TO ANGLE
    void MoveToAngle (double ang){
        //turning clockwise subtracts values
        //turning counter-clockwise adds values
        if (x > ang - 5 && x < ang + 5) {
            //has reached angle therefore end loop
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            turnCompleted = true;
            return;
        } else if (x < 5 + ang) {
            //turn clockwise
            FR.setPower(-.25);
            FL.setPower(.25);
            BR.setPower(-.25);
            BL.setPower(.25);
        } else if (x > ang - 5) {
            //turn counter-clockwise
            FR.setPower(0.25);
            FL.setPower(-.25);
            BR.setPower(0.25);
            BL.setPower(-.25);
        }
    }

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
