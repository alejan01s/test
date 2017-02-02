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

@Autonomous (name = "AutonomousRedNOTURN", group = "Sensor")
public class autonomousRedNoTurn extends LinearOpMode {

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
    public OpticalDistanceSensor colorOD;

    public Servo BallG1;
    public Servo BallG2;

    public boolean isRed;
    public boolean isBlue;

    //BUTTON PUSHER
    public Servo buttonPusher;
    public boolean buttonPress;
    public boolean buttonInit;
    public boolean push;

    public double NumberOfRevs3;

    public double colorCheckStep = 0;

    public boolean pushed = false;
    public boolean OppPushSequence = false;
    public boolean nearPush = false;
    public boolean shoot1;
    public boolean fired;

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
        colorOD = hardwareMap.opticalDistanceSensor.get("backOD");

        //BUTTON PUSHER
        buttonPusher = hardwareMap.servo.get("buttonPusher");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        buttonInit = false;
        buttonPress = false;
        push = false;
    }

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
        int NumberOfRevs1 = -300;
        int NumberOfRevs2 = -1600;

        //ANGLE VARIABLES
        double Angle1 = 190;
        double Angle2 = 280;

        imuTest imu = new imuTest("imu", hardwareMap);

        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()){

            bottomOD.enableLed(true);

            colorSensor.enableLed(false);
            isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
            isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("colorOD: ", colorOD.getRawLightDetected());
            telemetry.update();

            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double pitch = angles[1];
            double roll = angles[2];
            double x = yaw;

            if(x < 0){
                x = x + 360;
            }

            //SEQUENCES
            BallG1.setPosition(0);
            BallG2.setPosition(1);

            //CODE FOR SIDEWAYS STRAFE
            if(false){
                FR.setPower(0);
                BR.setPower(1);
                FL.setPower(1);
                BL.setPower(0);
                sleep(1500);
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
            }
            //MOVE FORWARD
            if(step == 0){
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
                    step=step+1;
                }
            }

            //LAUNCH BALLS
            if(step == 1){
                if(!fired) {
                    shoot1 = true;
                }
                if(fired) {
                    step = step + 1;
                }
            }
            if(step == 2){
                if(!shoot) {
                    shoot = true;
                    step=step+1;
                }
            }

            //Move forward
            if(step == 3){
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
            if(shoot1){
                if (LauncherM.getCurrentPosition() <= 2000) {
                    LauncherM.setPower(1);
                }
                else if (LauncherM.getCurrentPosition() <= 2450) {
                    LauncherM.setPower(.08);
                }
                else{
                    LauncherM.setPower(0);
                    fired = true;
                    shoot1 = false;
                    EncoderClicks = EncoderClicks + 2510;
                }
            }
            //Strafe for time
            if(step == 4){
                FR.setPower(-.75);
                BR.setPower(.75);
                FL.setPower(.75);
                BL.setPower(-.75);
                Thread.sleep(1600);
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                step=step+1;
            }

            //move to line
            if(step == 5){
                if(bottomOD.getRawLightDetected() < .08){
                    FL.setPower(-.4);
                    BL.setPower(-.4);
                    FR.setPower(-.4);
                    BR.setPower(-.4);
                }
                else{
                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    step=step+.5;
                }
            }
            if(step == 5.5){/*
                if (x > .5 && x < 1.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                    step=step+.5;
                } else if (x < .5) {
                    //turn clockwise
                    FR.setPower(-.5);
                    FL.setPower(.5);
                    BR.setPower(-.5);
                    BL.setPower(.5);
                } else if (x > 1.5) {
                    //turn counter-clockwise
                    FR.setPower(0.5);
                    FL.setPower(-.5);
                    BR.setPower(0.5);
                    BL.setPower(-.5);
                }*/
                step = step +.5;
            }
            //set revs3
            if(step == 6){
                if(colorOD.getRawLightDetected() < .0385) {
                    FR.setPower(-.1);
                    BR.setPower(.1);
                    FL.setPower(.1);
                    BL.setPower(-.1);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                }
                if(turnCompleted){
                    step = step + .5;
                    turnCompleted = false;
                }
            }
            if(step == 6.5){
                NumberOfRevs3 = FL.getCurrentPosition() - 50;
                step=step+.25;
            }
            //Position
            if(step == 6.75){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.25);
                    BR.setPower(-.25);
                    FR.setPower(-.25);
                    FL.setPower(-.25);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step=step+.25;
                }
            }

            //set possible rev3
            if(step == 7){
                NumberOfRevs3 = FL.getCurrentPosition() - 300;
                step=step+1;
            }

            //Detect color
            if(step == 8){
                isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                isBlue = colorSensor.blue() >= 2 && colorSensor.blue() > colorSensor.red() ? true : false;
                if(isBlue && !OppPushSequence){
                    //push button
                    nearPush = true;
                }
                else if(isRed && !nearPush){
                    //move forward confirm and push button
                    OppPushSequence = true;
                }
                if(nearPush){
                    if(!pushed) {
                        push = true;
                    }
                    else {
                        step = step + 1;
                    }
                }
                else if(OppPushSequence){
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
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + 1;
                        }
                        /*if(isBlue) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + 1;
                            }
                        }
                        else{
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
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + 1;
                                }
                            }
                        }*/
                    }
                }
            }

            //set next rev3
            if(step == 9){
                pushed = false;
                nearPush = false;
                OppPushSequence = false;
                NumberOfRevs3 = FL.getCurrentPosition() - 500;
                step=step+1;
            }

            //move forward
            if(step == 10){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.4);
                    BR.setPower(-.4);
                    FR.setPower(-.4);
                    FL.setPower(-.4);
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
                if(bottomOD.getRawLightDetected() < .08){
                    FL.setPower(-.4);
                    BL.setPower(-.4);
                    FR.setPower(-.4);
                    BR.setPower(-.4);
                }
                else{
                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    step=step+.25;
                }
            }
            if(step == 11.25){
                /*if (x > .5 && x < 1.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                    step=step+.25;
                } else if (x < .5) {
                    //turn clockwise
                    FR.setPower(-.5);
                    FL.setPower(.5);
                    BR.setPower(-.5);
                    BL.setPower(.5);
                } else if (x > 1.5) {
                    //turn counter-clockwise
                    FR.setPower(0.5);
                    FL.setPower(-.5);
                    BR.setPower(0.5);
                    BL.setPower(-.5);
                }*/
                step = step + .25;
            }
            if(step == 11.5){
                if(colorOD.getRawLightDetected() < .0395) {
                    FR.setPower(-.1);
                    BR.setPower(.1);
                    FL.setPower(.1);
                    BL.setPower(-.1);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                }
                if(turnCompleted){
                    step = step + .5;
                    turnCompleted = false;
                }
            }
            //set revs3
            if(step == 12){
                NumberOfRevs3 = FL.getCurrentPosition() - 25;
                step=step+1;
            }

            //position
            if(step == 13){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.25);
                    BR.setPower(-.25);
                    FR.setPower(-.25);
                    FL.setPower(-.25);
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
                NumberOfRevs3 = FL.getCurrentPosition() - 300;
                step=step+1;
            }

            //Detect color
            if(step == 15){
                isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                isBlue = colorSensor.blue() >= 2 && colorSensor.blue() > colorSensor.red() ? true : false;
                if(isBlue && !OppPushSequence){
                    //push button
                    nearPush = true;
                }
                else if(isRed && !nearPush){
                    //move forward confirm and push button
                    OppPushSequence = true;
                }
                if(nearPush){
                    if(!pushed) {
                        push = true;
                    }
                    else {
                        step = step + .5;
                    }
                }
                else if(OppPushSequence){
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
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + .5;
                        }
                        /*if(isBlue) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + 1;
                            }
                        }
                        else{
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
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + 1;
                                }
                            }
                        }*/
                    }
                }
            }
            if(step == 15.5){
                FR.setPower(.2);
                BR.setPower(-.2);
                FL.setPower(-.2);
                BL.setPower(.2);
                Thread.sleep(200);
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                step=step+.25;
            }
            if(step == 15.75){
                NumberOfRevs3 = FL.getCurrentPosition() - 540;
                step = step + .25;
            }
            //TURN
            if(step == 16){
                /*
                pushed = false;
                nearPush = false;
                OppPushSequence = false;
                if (x > 129 && x < 131) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                    step=step+1;
                } else if (x < 129) {
                    //turn clockwise
                    FR.setPower(-.5);
                    FL.setPower(.5);
                    BR.setPower(-.5);
                    BL.setPower(.5);
                } else if (x > 131) {
                    //turn counter-clockwise
                    FR.setPower(0.5);
                    FL.setPower(-.5);
                    BR.setPower(0.5);
                    BL.setPower(-.5);
                }*/
                if (FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.5);
                    BR.setPower(.5);
                    FR.setPower(.5);
                    FL.setPower(-.5);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }

            //set rev3
            if(step == 17){
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 3500;
                step=step+1;
            }

            //move forward
            if(step == 18){
                if(FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(1);
                    BR.setPower(1);
                    FR.setPower(1);
                    FL.setPower(1);
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
                sleep(1500);
                buttonPusher.setPosition(.6);
                sleep(1500);
                buttonInit = false;
                push = false;
                pushed = true;
            }

            /*

            SHOOTING SYSTEM

             */

            if(shoot) {
                if (!resume) {
                    if (LauncherM.getCurrentPosition() <= 400 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                    } else if (LauncherM.getCurrentPosition() <= 600 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                    } else {
                        pause = true;
                    }
                    if (pause) {
                        //INPUT RELOAD FUNCTION WHEN READY HERE
                        LauncherM.setPower(0.1);
                        Reloader.setPosition(.65);
                        resume = true;
                        pause = false;
                    }
                }
                if (resume) {
                    if (LauncherM.getCurrentPosition() > 600 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 1150 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.1);
                    } else if (LauncherM.getCurrentPosition() > 1150 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= 2255 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0.1);
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
        }
    }
    /*
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
    */
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
    /*
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
    */
}
