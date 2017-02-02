package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

    public boolean turnCompleted = false;

    //SENSORS
    public ColorSensor colorSensor;
    public OpticalDistanceSensor bottomOD;
    public OpticalDistanceSensor frontOD;
    public OpticalDistanceSensor colorOD;
    //public GyroSensor gyro;

    public boolean isRed;
    public boolean isBlue;

    public boolean shoot1;

    //BUTTON PUSHER
    public Servo buttonPusher;
    public boolean buttonPress;
    public boolean buttonInit;
    public boolean push;

    public double NumberOfRevs3;

    public double colorCheckStep = 0;

    public boolean hasStarted = false;
    public boolean pushed = false;

    public boolean stopAtLine = false;
    public boolean nearPush = false;
    public boolean OppPushSequence = false;
    public boolean OppPushSequence2 = false;

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

        //SEQUENCE VARIABLE
        double step = 0;

        //REVOLUTION VARIABLES
        int NumberOfRevs1 = -200;
        int NumberOfRevs2 = -1850;

        //ANGLE VARIABLES
        double Angle1 = 180;
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

            telemetry.addData("OD: ", bottomOD.getRawLightDetected());
            telemetry.addData("colorOD", colorOD.getRawLightDetected());
            telemetry.addData("RGB: ", colorSensor.argb());
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Blue: ", colorSensor.blue());

            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double pitch = angles[1];
            double roll = angles[2];
            double x = yaw;

            if(x < 0){
                x = x + 360;
            }
            // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
            telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.addData("X: ", x);
            telemetry.update();

            BallG1.setPosition(0);
            BallG2.setPosition(1);

            //SEQUENCES

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
                    step = step + 1;
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
                        shoot = false;
                    }
                }
                idle();
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

            //Strafe for time
            if(step == 4){
                Thread.sleep(100);
                if(!hasStarted) {
                    strafeLeft(1400);
                    hasStarted = true;
                }
                if(turnCompleted){
                    step = step + .15;
                    turnCompleted = false;
                    hasStarted = false;
                }
            }

            if(step == 4.15){
                sleep(100);
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 2100;
                step = step + .1;
            }

            if(step == 4.25){
                if (FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(1);
                    BR.setPower(-1);
                    FR.setPower(-1);
                    FL.setPower(1);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + .25;
                }
            }

            if(step == 4.5){
                if (x > 178.5 && x < 179.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                } else if (x < 178.5) {
                    //turn clockwise
                    FR.setPower(-.2);
                    FL.setPower(.2);
                    BR.setPower(-.2);
                    BL.setPower(.2);
                } else if (x > 179.5) {
                    //turn counter-clockwise
                    FR.setPower(0.2);
                    FL.setPower(-.2);
                    BR.setPower(0.2);
                    BL.setPower(-.2);
                }
                if(turnCompleted){
                    step = step + 1;
                }
            }
            if(step == 5.5){
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 75;
                step = step + .5;
            }
            //Strafe for time
            if(step == 6){
                stopAtLine(1);
                if(turnCompleted) {
                    step = step + .25;
                    turnCompleted = false;
                }
            }
            if(step == 6.25){
                if (x > 178.5 && x < 179.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                } else if (x < 178.5) {
                    //turn clockwise
                    FR.setPower(-.2);
                    FL.setPower(.2);
                    BR.setPower(-.2);
                    BL.setPower(.2);
                } else if (x > 179.5) {
                    //turn counter-clockwise
                    FR.setPower(0.2);
                    FL.setPower(-.2);
                    BR.setPower(0.2);
                    BL.setPower(-.2);
                }
                if(turnCompleted){
                    step = step + .25;
                }
            }
            if(step == 6.5){
                turnCompleted = false;
                Thread.sleep(5);
                hasStarted = false;
                step = step + .5;
            }
            //Move to line
            if(step == 7){
                if(colorOD.getRawLightDetected() < .0385) {
                    FR.setPower(.25);
                    BR.setPower(-.25);
                    FL.setPower(-.25);
                    BL.setPower(.25);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                }
                if(turnCompleted){
                    step = step + 1;
                    turnCompleted = false;
                    hasStarted = false;
                }
            }
            //Set revs3
            if(step == 8){
                Thread.sleep(100);
                turnCompleted = false;
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
                sleep(100);
                NumberOfRevs3 = FL.getCurrentPosition() + 285;
                step=step+1;
            }

            //Detect color
            if(step == 11){
                isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;
                if(isRed && !OppPushSequence){
                    //push button
                    nearPush = true;
                }
                else if(isBlue && !nearPush){
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
                        sleep(5);
                        //isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                        //isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + 1;
                        }
                        /*
                        if(isRed) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + 1;
                            }
                        }
                        else{
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
                                sleep(200);
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + 1;
                                }
                            }
                        }
                        */
                    }
                }
            }
            /*
            if(step == 12){
                if (x > 181.5 && x < 182.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                } else if (x < 181.5) {
                    //turn clockwise
                    FR.setPower(.2);
                    FL.setPower(-.2);
                    BR.setPower(.2);
                    BL.setPower(-.2);
                } else if (x > 182.5) {
                    //turn counter-clockwise
                    FR.setPower(-0.2);
                    FL.setPower(.2);
                    BR.setPower(-0.2);
                    BL.setPower(.2);
                }
                if(turnCompleted){
                    step = step + .5;
                }
            }
            */
            //set next rev3
            if(step == 12){
                turnCompleted = false;
                nearPush = false;
                OppPushSequence = false;
                pushed = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 750;
                step=step+1;
            }

            //move forward
            if(step == 13){
                if(FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(.4);
                    BR.setPower(.4);
                    FR.setPower(.4);
                    FL.setPower(.4);
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
                stopAtLine2(1);
                if(turnCompleted){
                    step=step+.5;
                    turnCompleted = false;
                }
            }
            if(step == 14.5){
                if (x > 178.5 && x < 179.5) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                } else if (x < 178.5) {
                    //turn clockwise
                    FR.setPower(-.1);
                    FL.setPower(.1);
                    BR.setPower(-.1);
                    BL.setPower(.1);
                } else if (x > 179.5) {
                    //turn counter-clockwise
                    FR.setPower(0.1);
                    FL.setPower(-.1);
                    BR.setPower(0.1);
                    BL.setPower(-.1);
                }
                if(turnCompleted) {
                    step = step + .1;
                }
            }
            if(step == 14.6){
                turnCompleted = false;
                step = step + .15;
            }
            if(step == 14.75){
                sleep(100);
                if(colorOD.getRawLightDetected() < .0395) {
                    FR.setPower(.25);
                    BR.setPower(-.25);
                    FL.setPower(-.25);
                    BL.setPower(.25);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                }
                if(turnCompleted){
                    step = step + .25;
                    turnCompleted = false;
                    hasStarted = false;
                }
            }
            //set revs3
            if(step == 15){
                turnCompleted = false;
                OppPushSequence = false;
                pushed = false;
                sleep(100);
                NumberOfRevs3 = FL.getCurrentPosition() - 550;
                step=step+1;
            }

            //position
            if(step == 16){
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
            if(step == 17){
                turnCompleted = false;
                OppPushSequence = false;
                pushed = false;
                hasStarted = false;
                sleep(100);
                NumberOfRevs3 = FL.getCurrentPosition() + 315;
                step=step+1;
            }

            //Detect color
            if(step == 18){
                isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;
                if(isRed && !OppPushSequence2){
                    //push button
                    nearPush = true;
                }
                else if(isBlue && !nearPush){
                    //move forward confirm and push button
                    OppPushSequence2 = true;
                }
                if(nearPush){
                    if(!pushed) {
                        push = true;
                    }
                    else {
                        step = step + 1;
                    }
                }
                if(OppPushSequence2){
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
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + .5;
                        }
                        //isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
                        //isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;
                        /*
                        if(isRed) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + .5;
                            }
                        }
                        else{
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
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + .5;
                                }
                            }
                        }
                        */
                    }
                }
            }
            if(step == 18.5){
                nearPush = false;
                OppPushSequence = false;
                pushed = false;
                sleep(100);
                NumberOfRevs3 = FL.getCurrentPosition() + 500;
                //add strafe
                step = step + .55;
            }
            //TURN
            if(step == 2000){
                if (x > 223 && x < 227) {
                    //has reached angle therefore end loop
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    turnCompleted = true;
                    step = step + 1;
                } else if (x < 223) {
                    //turn clockwise
                    FR.setPower(-.4);
                    FL.setPower(.4);
                    BR.setPower(-.4);
                    BL.setPower(.4);
                } else if (x > 227) {
                    //turn counter-clockwise
                    FR.setPower(0.4);
                    FL.setPower(-.4);
                    BR.setPower(0.4);
                    BL.setPower(-.4);
                }
                //super.runOpMode(Angle2, false);
            }

            //set rev3
            if(step == 20){
                sleep(100);
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() - 3000;
                step=step+1;
            }

            //move forward
            if(step == 21){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-1);
                    BR.setPower(-1);
                    FR.setPower(-1);
                    FL.setPower(-1);
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
                sleep(1300);
                buttonPusher.setPosition(.6);
                sleep(1300);
                buttonInit = false;
                push = false;
                pushed = true;
            }
        }
    }
    //detect color
    /*
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
            BL.setPower(.45);
            BR.setPower(.45);
            FR.setPower(.45);
            FL.setPower(.45);
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
        if(bottomOD.getRawLightDetected() < .08){
            FL.setPower(.4 * dir);
            BL.setPower(.4 * dir);
            FR.setPower(.4 * dir);
            BR.setPower(.4 * dir);
        }
        else{
            if(!turnCompleted) {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                turnCompleted = true;
            }
            return;
        }
    }
    void stopAtLine2 (int dir) {
        if(bottomOD.getRawLightDetected() < .08){
            FL.setPower(.4 * dir);
            BL.setPower(.4 * dir);
            FR.setPower(.4 * dir);
            BR.setPower(.4 * dir);
        }
        else{
            if(!turnCompleted) {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                turnCompleted = true;
            }
            return;
        }
    }
    //strafe left
    void strafeLeft (int time) {
        FR.setPower(-1);
        BR.setPower(1);
        FL.setPower(1);
        BL.setPower(-1);
        sleep(time);
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        turnCompleted = true;
        return;
    }
    void strafeRight (int time) {
        if(colorOD.getRawLightDetected() < .07) {
            FR.setPower(.25);
            BR.setPower(-.25);
            FL.setPower(-.25);
            BL.setPower(.25);
        }
        else {
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            turnCompleted = true;
            return;
        }
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
}
