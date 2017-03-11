
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aleja on 12/4/2016.
 */

//Plan: Launch ball into goal, reload, launch second, then hit beacons and then knock the ball

@Autonomous (name = "Primary Red Center", group = "Sensor")
public class autonomousFullRed extends OpMode {

    /*

    I2C GLOBAL VARIABLES

     */
    //Defining Color Sensors
    I2cDevice ColorR;
    I2cDevice ColorL;
    I2cDeviceSynch ColorRReader;
    I2cDeviceSynch ColorLReader;

    //Defining Color Sensor Variables
    byte[] ColorRNumber;
    byte[] ColorLNumber;
    double CRNumber;
    double CLNumber;

    byte[] ColorRRed;
    byte[] ColorLRed;
    double CRRed;
    double CLRed;

    byte[] ColorRGreen;
    byte[] ColorLGreen;
    double CRGreen;
    double CLGreen;

    byte[] ColorRBlue;
    byte[] ColorLBlue;
    double CRBlue;
    double CLBlue;

    byte[] ColorRWhite;
    byte[] ColorLWhite;
    double CRWhite;
    double CLWhite;

    boolean ColorReadGamepadToggle;



    //Defining Sonar Sensors
    I2cDevice SonarR;
    I2cDevice SonarL;
    I2cDeviceSynch SonarRReader;
    I2cDeviceSynch SonarLReader;

    //Defining Sonar Sensor Variables
    byte[] RightDistanceTimeH;
    byte[] LeftDistanceTimeH;
    byte[] RightDistanceTimeL;
    byte[] LeftDistanceTimeL;
    double RightDistanceTime;
    double LeftDistanceTime;
    double RightDistanceTimeHDouble;
    double RightDistanceTimeLDouble;
    double LeftDistanceTimeHDouble;
    double LeftDistanceTimeLDouble;

    byte[] RightDistanceIN;
    byte[] LeftDistanceIN;
    double RightIN;
    double LeftIN;

    byte[] RightDistanceCM;
    byte[] LeftDistanceCM;
    double RightCM;
    double LeftCM;

    //Switches for Taking Distance Snapshots
    boolean CollectDistanceTime = false;
    boolean CollectDistanceIN = false;
    boolean CollectDistanceCM = false;

    //Defining Sleep Method Variables
    double Sleep = 0;
    double WakeUpTime;
    double SleepEnable = 0;

    /*
    ---------------------------------------------------------------------------------------------------
    AUTONOMOUS VARIABLES

     */
    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //VARIABLES FOR LAUNCHER
    public double EncoderClicks;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;
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
    public Servo buttonPusher2;

    public boolean buttonPress;
    public boolean buttonInit;
    public boolean push;

    public double NumberOfRevs5 = 0;
    public double NumberOfRevs4;
    public double NumberOfRevs3;

    public double colorCheckStep = 0;

    public boolean pushed = false;
    public boolean OppPushSequence = false;
    public boolean nearPush = false;
    public boolean shoot1;
    public boolean fired;

    boolean lookEnable = false;
    boolean recordRealVal = false;

    public boolean firstCollect = true;

    public boolean launcherCorrect = true;

    imuTest imu;

    @Override
    public void init () {
        /*

        I2CC INIT

         */

        //Setting Up the Color Sensors

        //Mapping Color Sensors
        ColorR = hardwareMap.i2cDevice.get("ColorR");
        ColorL = hardwareMap.i2cDevice.get("ColorL");

        //Telling the Robot Which I2C Address to Talk To
        ColorRReader = new I2cDeviceSynchImpl(ColorR, I2cAddr.create8bit(0x3a), false);
        ColorLReader = new I2cDeviceSynchImpl(ColorL, I2cAddr.create8bit(0x3c), false);

        //Turning On Color Sensors
        ColorRReader.engage();
        ColorLReader.engage();



        //Setting Up the Sonar Sensors

        //Mapping Sonar Sensors
        SonarR = hardwareMap.i2cDevice.get("SonarR");
        SonarL = hardwareMap.i2cDevice.get("SonarL");

        //Telling the Robot Which I2C Address to Talk To
        SonarRReader = new I2cDeviceSynchImpl(SonarR, I2cAddr.create8bit(0xe0), false);
        SonarLReader = new I2cDeviceSynchImpl(SonarL, I2cAddr.create8bit(0xe2), false);

        //Turning on Sonar Sensors
        SonarRReader.engage();
        SonarLReader.engage();

        //Below are the range and gain settings for the sonar sensors

        //Right Sonar Sensor Configuration
        SonarRReader.write8(1, 3);//Gain
        SonarRReader.write8(2, 6);//Range

        //Left Sonar Sensor Configuration
        SonarLReader.write8(1, 3);//Gain
        SonarLReader.write8(2, 6);//Range



        //Putting the Robot to Sleep for 200ms to Ensure Settings are Written to Sonar Sensors
        Sleep = 200;

        if(SleepEnable == 0)
        {

            WakeUpTime = System.currentTimeMillis() + Sleep;

            while(System.currentTimeMillis() < WakeUpTime)
            {

            }

            Sleep = 0;

            SleepEnable = 1;

        }

        //Notifying Operator that the Initialization Routine has Finished
        telemetry.addLine("Initialization Complete");


        /*
        ---------------------------------------------------------------------
        AUTONOMOUS INIT

         */
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
        bottomOD = hardwareMap.opticalDistanceSensor.get("bottomOD");
        frontOD = hardwareMap.opticalDistanceSensor.get("backOD");
        colorOD = hardwareMap.opticalDistanceSensor.get("frontOD");

        //BUTTON PUSHER

        buttonPusher = hardwareMap.servo.get("buttonPusher2");
        buttonPusher2 = hardwareMap.servo.get("buttonPusher");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        buttonInit = false;
        buttonPress = false;
        push = false;

        imu = new imuTest("imu", hardwareMap);
    }
    //SEQUENCE VARIABLE
    double step = 0;
    double ButtonPusherExtensionStep = -1;
    double ButtonPusherRetractionStep = -1;
    boolean StartPush = false;
    boolean PusherExtend = false;
    boolean PusherRetract = false;
    boolean EndPush = false;
    boolean Push1 = false;
    boolean Push1Go = false;
    boolean Push1End = false;
    boolean Push1Complete = false;
    boolean Push2 = false;
    boolean Push2Go = false;
    boolean Push2End = false;
    boolean Push2Complete = false;
    boolean Push3 = false;
    boolean Push3Go = false;
    boolean Push3End = false;
    boolean Push3Complete = false;
    boolean Push4 = false;
    boolean Push4Go = false;
    boolean Push4End = false;
    boolean Push4Complete = false;
    boolean Push5 = false;
    boolean Push5Go = false;
    boolean Push5End = false;
    boolean Push5Complete = false;
    boolean Push6 = false;
    boolean Push6Go = false;
    boolean Push6End = false;
    boolean Push6Complete = false;
    boolean Ignore = false;
    double PusherSleep = 0;

    //REVOLUTION VARIABLES
    int NumberOfRevs1 = -170;
    int NumberOfRevs2 = -900;

    //ANGLE VARIABLES
    double Angle1 = 190;
    double Angle2 = 280;

    //PRACTICE VALUES: .035, .05
    //USRA VALUES: .075, .08
    double beaconOneDistance = .053;
    double beaconTwoDistance = .056;

    boolean sleepOn = false;
    double timeToSleep;
    double timeToWake;

    double rightDisIN;
    double leftDisIN;
    double rightDisCM;
    double leftDisCM;
    double rightDisMS;
    double leftDisMS;

    boolean runCheck = false;

    @Override
    public void start () {
        //telemetry.addData("Status", "Initialization Complete");
        //telemetry.update();

        launcherCorrect = false;
        EncoderClicks = LauncherM.getCurrentPosition() + 2520;
    }

    @Override
    public void loop (){

            /*

            I2C LOOP

             */

        boolean StartRead = false;
        boolean LookForValue = false;
        boolean RecordValue = false;

        //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
        RightDistanceTimeH = SonarRReader.read(0x02, 1);
        RightDistanceTimeL = SonarRReader.read(0x03, 1);

        //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
        LeftDistanceTimeH = SonarLReader.read(0x02, 1);
        LeftDistanceTimeL = SonarLReader.read(0x03, 1);


        //Reading the Color Sensors

        //Obtaining the Red Value
        ColorRRed = ColorRReader.read(0x05, 1);
        ColorLRed = ColorLReader.read(0x05, 1);

        CRRed = ColorRRed[0] & 0xFF;
        CLRed = ColorLRed[0] & 0xFF;

        //Obtaining the Blue Value
        ColorRBlue = ColorRReader.read(0x07, 1);
        ColorLBlue = ColorLReader.read(0x07, 1);

        CRBlue = ColorRBlue[0] & 0xFF;
        CLBlue = ColorLBlue[0] & 0xFF;

//            //Reading the Sonar Sensors
//
//            //Reading Distance in MicroSeconds
//            if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
//            {
//
//                StartRead = true;
//
//
//                CollectDistanceTime = false;
//            }
//
//            if(StartRead == true) {
//                //Command the Sonars to Take a Snapshot
//                SonarRReader.write8(0, 82);
//                SonarLReader.write8(0, 82);
//
//                LookForValue = true;
//                StartRead = false;
//
//            }
//
//            if(LookForValue == true)
//            {
//
//                //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
//                RightDistanceTimeH = SonarRReader.read(0x02, 1);
//                RightDistanceTimeL = SonarRReader.read(0x03, 1);
//                RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
//                RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;
//
//                //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
//                LeftDistanceTimeH = SonarLReader.read(0x02, 1);
//                LeftDistanceTimeL = SonarLReader.read(0x03, 1);
//                LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
//                LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;
//
//            }
//
//            if(((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255)))
//            {
//
//                LookForValue = false;
//                RecordValue = true;
//
//            }
//
//            if(RecordValue == true) {
//
//                //Save Full Distance Values from Last Snapshot
//                RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
//                LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
//            }

            /*
            -------------------------------------------------------
            AUTONOMOUS LOOP

             */
        bottomOD.enableLed(true);
        frontOD.enableLed(true);

        if(firstCollect){
            CollectDistanceTime = true;
            firstCollect = false;
        }

        //colorSensor.enableLed(false);
        //isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
        //isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;

        isRed = CLRed > CLBlue && CLRed >= 1 ? true : false;
        isBlue = CLBlue > CLRed && CLBlue >= 1 ? true : false;

        telemetry.addData("Current Time: ", System.currentTimeMillis());
        telemetry.addData("PusherSleep: ", PusherSleep);
        telemetry.addData("Push1Go: ", Push1Go);
        telemetry.addData("Push1End: ", Push1End);
        telemetry.addData("Launcher", LauncherM.getCurrentPosition());
        telemetry.addData("Encoder Clicks: ", (EncoderClicks - 2520));

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());

        telemetry.addData("colorOD: ", colorOD.getRawLightDetected());
        telemetry.addData("Red: ", CLRed);
        telemetry.addData("Blue: ", CLBlue);

        telemetry.addData("Left Distance Time: ", LeftDistanceTime);
        telemetry.addData("Bottom OD: ", bottomOD);

        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double pitch = angles[1];
        double roll = angles[2];
        double x = yaw;

        if(x < 0){
            x = x + 360;
        }

        telemetry.addData(imu.getName(), imu.telemetrize());
        telemetry.addData("X: ", x);
        telemetry.update();

        //SEQUENCES
        BallG1.setPosition(0);
        BallG2.setPosition(1);
        buttonPusher2.setPosition(.5);

        //MOVE FORWARD
        if(step == 0){

            if(!firstCollect) {
                if (FL.getCurrentPosition() > NumberOfRevs1) {
                    BL.setPower(-.35);
                    BR.setPower(-.35);
                    FR.setPower(-.35);
                    FL.setPower(-.35);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 0.5;
                }
            }
        }

        if(step == 0.5)
        {

            Push1 = true;
            step = step + 0.5;

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

            Push1Complete = false;

            if(!shoot) {
                if (FL.getCurrentPosition() > NumberOfRevs2) {
                    BL.setPower(-.35);
                    BR.setPower(-.35);
                    FR.setPower(-.35);
                    FL.setPower(-.35);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);

                    NumberOfRevs5 = FR.getCurrentPosition() - 4000;
                    NumberOfRevs4 = FR.getCurrentPosition() - 3500;
                    NumberOfRevs3 = FR.getCurrentPosition() - 4650;
                    step = step + 0.1;
                }
            }
        }
        if(shoot1){
            if (LauncherM.getCurrentPosition() <= EncoderClicks - 500) {
                LauncherM.setPower(0.85);
            }
            else{
                LauncherM.setPower(0);
                fired = true;
                shoot1 = false;
                EncoderClicks = EncoderClicks + 2520;
            }
        }

        //Strafe for time
        if(step == 3.1){

            if(!Push1Complete) {
                Push1 = true;
            }
                /*if (bottomOD.getRawLightDetected() < .08) {
                    FR.setPower(-1);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(-1);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    step = step + 1;
                }*/

            if(FR.getCurrentPosition() > NumberOfRevs4){
                FR.setPower(-1);
                BL.setPower(-1);
            }
            else if(FR.getCurrentPosition() > NumberOfRevs5)
            {

                FR.setPower(-0.65);
                BL.setPower(-0.65);

            }
            else if(FR.getCurrentPosition() > NumberOfRevs3)
            {

                FR.setPower(-0.3);
                BL.setPower(-0.3);

            }
            else {
                FR.setPower(0);
                //BR.setPower(0);
                //FL.setPower(0);
                BL.setPower(0);
                step = step + 1.9;
            }
        }

        //Strafe for time
        if(step == 4){
                /*if (bottomOD.getRawLightDetected() < .08) {
                    FR.setPower(-1);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(-1);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    step = step + 1;
                }*/

            if(FR.getCurrentPosition() > NumberOfRevs3){
                FR.setPower(-0.5);
                BL.setPower(-0.5);
            }
            else{
                FR.setPower(0);
                BL.setPower(0);
                step = step + 1;
            }
        }

        if(step == 5){
//                if (x > .5 && x < 1.5) {
//                    //has reached angle therefore end loop
//                    FR.setPower(0);
//                    FL.setPower(0);
//                    BR.setPower(0);
//                    BL.setPower(0);
//                    turnCompleted = true;
//                    step=step+.25;
//                } else if (x < .5) {
//                    //turn clockwise
//                    FR.setPower(-.05);
//                    FL.setPower(.05);
//                    BR.setPower(-.05);
//                    BL.setPower(.05);
//                } else if (x > 1.5) {
//                    //turn counter-clockwise
//                    FR.setPower(0.05);
//                    FL.setPower(-.05);
//                    BR.setPower(0.05);
//                    BL.setPower(-.05);
//                }
            while(yaw < -1.5 || yaw > 0){
                angles = imu.getAngles();
                yaw = angles[0];
                if(yaw < -1.5){

                    //turn clockwise
                    FR.setPower(-.2);
                    FL.setPower(.2);
                    BR.setPower(-.2);
                    BL.setPower(.2);

                }
                else if(yaw > 0){

                    //turn counter-clockwise
                    FR.setPower(0.2);
                    FL.setPower(-.2);
                    BR.setPower(0.2);
                    BL.setPower(-.2);

                }
            }

            //has reached angle therefore end loop
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            step=step+.5;
//
//                if (yaw > -3.5 && yaw < 0) {
//                    //has reached angle therefore end loop
//                    FR.setPower(0);
//                    FL.setPower(0);
//                    BR.setPower(0);
//                    BL.setPower(0);
//                    step=step+.5;
//                } else if (yaw < -3.5) {
//                    //turn clockwise
//                    FR.setPower(-.1);
//                    FL.setPower(.1);
//                    BR.setPower(-.1);
//                    BL.setPower(.1);
//                } else if (yaw > 0) {
//                    //turn counter-clockwise
//                    FR.setPower(0.1);
//                    FL.setPower(-.1);
//                    BR.setPower(0.1);
//                    BL.setPower(-.1);
//                }
        }


        //move to line
        if(step == 5.5){
                /*
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
                */
            while (bottomOD.getRawLightDetected() < .04) {

                bottomOD.getLightDetected();

                FR.setPower(.2);
                BR.setPower(.2);
                FL.setPower(.2);
                BL.setPower(.2);
            }

            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            step = step + .25;

        }


        if(step == 5.75){
            turnCompleted = false;
            NumberOfRevs3 = FL.getCurrentPosition() + 60;
            step = step + .2;
        }
        if(step == 5.95) {
            if (FL.getCurrentPosition() < NumberOfRevs3) {
                BL.setPower(.1);
                BR.setPower(.1);
                FR.setPower(.1);
                FL.setPower(.1);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                FL.setPower(0);
                launcherCorrect = false;
                LeftDistanceTime = 10000;
                step = step + .05;
            }
        }
        //set revs3
        if(step == 6){
            while(LeftDistanceTime > 1075 || LeftDistanceTime == 0) {
                CollectDistanceTime = true;

                //Reading the Sonar Sensors

                //Reading Distance in MicroSeconds
                if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
                {

                    StartRead = true;


                    CollectDistanceTime = false;
                }

                if(StartRead == true) {
                    //Command the Sonars to Take a Snapshot
                    SonarRReader.write8(0, 82);
                    SonarLReader.write8(0, 82);

                    LookForValue = true;
                    StartRead = false;

                }

                if(LookForValue == true)
                {

                    //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
                    RightDistanceTimeH = SonarRReader.read(0x02, 1);
                    RightDistanceTimeL = SonarRReader.read(0x03, 1);
                    RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
                    RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;

                    //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
                    LeftDistanceTimeH = SonarLReader.read(0x02, 1);
                    LeftDistanceTimeL = SonarLReader.read(0x03, 1);
                    LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
                    LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;

                }

                if(((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255)))
                {

                    LookForValue = false;
                    RecordValue = true;

                }

                if(RecordValue == true) {

                    //Save Full Distance Values from Last Snapshot
                    RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
                    LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
                }


                FR.setPower(-.1);
                BR.setPower(.1);
                FL.setPower(.1);
                BL.setPower(-.1);
            }


            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            CollectDistanceTime = false;
            step = step + 0.5;


        }
        if(step == 6.5){
            NumberOfRevs3 = FL.getCurrentPosition() - 30;
            launcherCorrect = true;
            CollectDistanceTime = false;
            step=step+.25;
        }
        //Position
        if(step == 6.75){
            if(FL.getCurrentPosition() > NumberOfRevs3) {
                BL.setPower(-.15);
                BR.setPower(-.15);
                FR.setPower(-.15);
                FL.setPower(-.15);
            }
            else {
                BL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                FL.setPower(0);
                NumberOfRevs4 = 0;
                NumberOfRevs3 = 0;
                step=step+.25;

            }

        }

        //set possible rev3
        if(step == 7){
            NumberOfRevs4 = FL.getCurrentPosition() - 45;
            NumberOfRevs3 = FL.getCurrentPosition() - 340;
            step=step+1;
        }

        //Detect color
        if(step == 8){
            isRed = CLRed > CLBlue && CLRed >= 1 ? true : false;
            isBlue = CLBlue > CLRed && CLBlue >= 1 ? true : false;
            if(isRed && !OppPushSequence){
                //push button
                nearPush = true;
            }
            else if(isBlue && !nearPush){
                //move forward confirm and push button
                OppPushSequence = true;
            }
            if(nearPush){

                if(FL.getCurrentPosition() > NumberOfRevs4)
                {

                    BL.setPower(-.15);
                    BR.setPower(-.15);
                    FR.setPower(-.15);
                    FL.setPower(-.15);

                }

                else {

                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);

//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//
//                        if (!pushed) {
//                            push = true;
//                        } else {
//                            step = step + 1;
//
//
//                        }

                    if(!Push3Complete)
                    {

                        Push2 = true;

                    }
                    else
                    {

                        step = step + 1;

                    }
                }

            }
            else if(OppPushSequence){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.15);
                    BR.setPower(-.15);
                    FR.setPower(-.15);
                    FL.setPower(-.15);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//                        if (!pushed) {
//                            push = true;
//                        }
//                        if (pushed) {
//                            step = step + 1;
//                        }
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

                    if(!Push3Complete)
                    {

                        Push2 = true;

                    }
                    else
                    {

                        step = step + 1;

                    }

                }
            }
        }

        //set next rev3
        if(step == 9){

            NumberOfRevs3 = FL.getCurrentPosition() - 1000;
            if(OppPushSequence)
            {

                NumberOfRevs3 = FL.getCurrentPosition() - 575;

            }
            step=step+1;
        }

        //move forward
        if(step == 10){
            pushed = false;
            nearPush = false;
            OppPushSequence = false;

            if(!Push5Complete) {
                Push5 = true;
            }

            if(FL.getCurrentPosition() > NumberOfRevs3 + 400) {
                BL.setPower(-1);
                BR.setPower(-1);
                FR.setPower(-1);
                FL.setPower(-1);
            }

            else if(FL.getCurrentPosition() > NumberOfRevs3 + 200)
            {

                BL.setPower(-.65);
                BR.setPower(-.65);
                FR.setPower(-.65);
                FL.setPower(-.65);

            }

            else if(FL.getCurrentPosition() > NumberOfRevs3)
            {

                BL.setPower(-.35);
                BR.setPower(-.35);
                FR.setPower(-.35);
                FL.setPower(-.35);

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

            launcherCorrect = false;

            while(bottomOD.getRawLightDetected() < .04){

                bottomOD.getLightDetected();

                FL.setPower(-.2);
                BL.setPower(-.2);
                FR.setPower(-.2);
                BR.setPower(-.2);
            }

            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            step=step+.25;

        }
        if(step == 11.25){

            while(yaw < -1.5 || yaw > 0){
                angles = imu.getAngles();
                yaw = angles[0];
                if(yaw < -1.5){

                    //turn clockwise
                    FR.setPower(-.2);
                    FL.setPower(.2);
                    BR.setPower(-.2);
                    BL.setPower(.2);

                }
                else if(yaw > 0){

                    //turn counter-clockwise
                    FR.setPower(0.2);
                    FL.setPower(-.2);
                    BR.setPower(0.2);
                    BL.setPower(-.2);

                }
            }

            //has reached angle therefore end loop
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            //step=step+.25;



//                if (yaw > -3.5 && yaw < 0) {
//                    //has reached angle therefore end loop
//                    FR.setPower(0);
//                    FL.setPower(0);
//                    BR.setPower(0);
//                    BL.setPower(0);
//                    step=step+.25;
//                } else if (yaw < -3.5) {
//                    //turn clockwise
//                    FR.setPower(-.1);
//                    FL.setPower(.1);
//                    BR.setPower(-.1);
//                    BL.setPower(.1);
//                } else if (yaw > 0) {
//                    //turn counter-clockwise
//                    FR.setPower(0.1);
//                    FL.setPower(-.1);
//                    BR.setPower(0.1);
//                    BL.setPower(-.1);
//                }


            turnCompleted = false;
            LeftDistanceTime = 10000;
            step = step + .25;
        }
        if(step == 11.5){
            while(LeftDistanceTime > 1075 || LeftDistanceTime == 0) {
                CollectDistanceTime = true;

                //Reading the Sonar Sensors

                //Reading Distance in MicroSeconds
                if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
                {

                    StartRead = true;


                    CollectDistanceTime = false;
                }

                if (StartRead == true) {
                    //Command the Sonars to Take a Snapshot
                    SonarRReader.write8(0, 82);
                    SonarLReader.write8(0, 82);

                    LookForValue = true;
                    StartRead = false;

                }

                if (LookForValue == true) {

                    //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
                    RightDistanceTimeH = SonarRReader.read(0x02, 1);
                    RightDistanceTimeL = SonarRReader.read(0x03, 1);
                    RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
                    RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;

                    //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
                    LeftDistanceTimeH = SonarLReader.read(0x02, 1);
                    LeftDistanceTimeL = SonarLReader.read(0x03, 1);
                    LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
                    LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;

                }

                if (((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255))) {

                    LookForValue = false;
                    RecordValue = true;

                }

                if (RecordValue == true) {

                    //Save Full Distance Values from Last Snapshot
                    RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
                    LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
                }


                FR.setPower(-.1);
                BR.setPower(.1);
                FL.setPower(.1);
                BL.setPower(-.1);

            }

            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            CollectDistanceTime = false;
            step = step + .5;

        }
        //set revs3
        if(step == 12){
            NumberOfRevs3 = FL.getCurrentPosition() - 10;
            step=step+2;
        }

        //position
        if(step == 13){
            if(FL.getCurrentPosition() > NumberOfRevs3) {
                BL.setPower(-.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                FL.setPower(-.2);
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
            NumberOfRevs3 = FL.getCurrentPosition() - 365;
            NumberOfRevs4 = FL.getCurrentPosition() - 55;
            step=step+1;
        }

        //Detect color
        if(step == 15){
            isRed = CLRed > CLBlue && CLRed >= 1 ? true : false;
            isBlue = CLBlue > CLRed && CLBlue >= 1 ? true : false;
            if(isRed && !OppPushSequence){
                //push button
                nearPush = true;
            }
            else if(isBlue && !nearPush){
                //move forward confirm and push button
                OppPushSequence = true;
            }
            if(nearPush){

                NumberOfRevs5 = 100;

                if(FL.getCurrentPosition() > NumberOfRevs4) {
                    BL.setPower(-.15);
                    BR.setPower(-.15);
                    FR.setPower(-.15);
                    FL.setPower(-.15);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);


//                        if(!pushed) {
//                            push = true;
//                        }
//                        else {
//                            step = step + .5;
//                        }

                    if(!Push4Complete)
                    {

                        Push4 = true;

                    }
                    else
                    {

                        step = step+0.5;

                    }


                }

            }
            else if(OppPushSequence){

                NumberOfRevs5 = 100;

                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-.15);
                    BR.setPower(-.15);
                    FR.setPower(-.15);
                    FL.setPower(-.15);
                }
                else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
/*                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + .5;
                        }*/
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

                    if(!Push4Complete)
                    {

                        Push4 = true;

                    }
                    else
                    {

                        step = step+0.5;

                    }
                }
            }
        }
        if(step == 15.5){

//                Ignore = true;
//

            buttonPusher.setPosition(0.4);

            FR.setPower(1);
            BR.setPower(-1);
            FL.setPower(-1);
            BL.setPower(1);
            sleepOn = true;
            timeToSleep = 200;

            if (sleepOn) {

                timeToWake = System.currentTimeMillis() + timeToSleep;

                while (System.currentTimeMillis() < timeToWake) {

                }

                timeToSleep = 0;

                sleepOn = false;

            }
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            step=step+.25;
        }
        if(step == 15.75){
            NumberOfRevs3 = FL.getCurrentPosition() + 270;
            if(OppPushSequence){
                NumberOfRevs3 = FL.getCurrentPosition() + 285;
            }
            step = step + .25;
        }
        //TURN
        if(step == 16){

            while(yaw < 32){
                angles = imu.getAngles();
                yaw = angles[0];

                FR.setPower(-.3);
                FL.setPower(.3);
                BR.setPower(-.3);
                BL.setPower(.3);

            }

            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

            step = step + 1;



                /*
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
//
//                if (FL.getCurrentPosition() < NumberOfRevs3) {
//                    BL.setPower(.5);
//                    BR.setPower(-.5);
//                    FR.setPower(-.5);
//                    FL.setPower(.5);
//                } else {
//                    BL.setPower(0);
//                    BR.setPower(0);
//                    FR.setPower(0);
//                    FL.setPower(0);
//                    step = step + 1;
//                }
        }

        //set rev3
        if(step == 17){
            turnCompleted = false;
            NumberOfRevs3 = FL.getCurrentPosition() + 2775;
            if(OppPushSequence)
            {

                NumberOfRevs3 = FL.getCurrentPosition() + 3200;

            }
            step=step+1;
        }

        //move forward
        if(step == 18){

            //Push6 = true;

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

//            if(step == 19)
//            {
//
//                Push6 = true;
//                step = step+1;
//
//            }

            /*

            Button Pusher

             */

        if(Push1 && Ignore == false)
        {

            Push1Complete = false;
            PusherSleep = System.currentTimeMillis() + 500;
            buttonPusher.setPosition(0.6);
            Push1Go = true;
            Push1 = false;

        }

        if(Push1Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push1End = true;
            Push1Go = false;
        }

        if(Push1End && Ignore == false)
        {


            Push1Complete = true;
            Push1End = false;

        }



        if(Push2 && Ignore == false)
        {

            Push2Complete = false;
            PusherSleep = System.currentTimeMillis() + 2400;
            buttonPusher.setPosition(0.6);
            Push2Go = true;
            Push2 = false;

        }

        if(Push2Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push2End = true;
            Push2Go = false;
        }

        if(Push2End && Ignore == false)
        {


            Push2Complete = true;
            Push2End = false;

        }

        if(Push2Complete && Ignore == false)
        {

            Push3 = true;
            Push2Complete = false;

        }



        if(Push3 && Ignore == false)
        {

            Push3Complete = false;
            PusherSleep = System.currentTimeMillis() + 1000;
            buttonPusher.setPosition(0.4);
            Push3Go = true;
            Push3 = false;

        }

        if(Push3Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push3End = true;
            Push3Go = false;
        }

        if(Push3End && Ignore == false)
        {


            Push3Complete = true;
            Push3End = false;

        }





        if(Push4 && Ignore == false)
        {

            Push4Complete = false;
            PusherSleep = System.currentTimeMillis() + 2000;
            buttonPusher.setPosition(0.6);
            Push4Go = true;
            Push4 = false;

        }

        if(Push4Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push4End = true;
            Push4Go = false;
        }

        if(Push4End && Ignore == false)
        {


            Push4Complete = true;
            Push4End = false;

        }

//            if(Push4Complete)
//            {
//
//                Push5 = true;
//                Push4Complete = false;
//
//            }



        if(Push5 && Ignore == false)
        {

            Push5Complete = false;
            PusherSleep = System.currentTimeMillis() + 500;
            buttonPusher.setPosition(0.4);
            Push5Go = true;
            Push5 = false;

        }

        if(Push5Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push5End = true;
            Push5Go = false;
        }

        if(Push5End && Ignore == false)
        {


            Push5Complete = true;
            Push5End = false;

        }



        if(Push6 && Ignore == false)
        {

            Push6Complete = false;
            PusherSleep = System.currentTimeMillis() + 2000;
            buttonPusher.setPosition(0.4);
            Push6Go = true;
            Push6 = false;

        }

        if(Push6Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {



            }

            buttonPusher.setPosition(0.5);
            Push6End = true;
            Push6Go = false;
        }

        if(Push6End && Ignore == false)
        {


            Push6Complete = true;
            Push6End = false;

        }







//            if(!buttonInit){
//                buttonPusher.setPosition(.5);
//                if(push){
//                    buttonInit = true;
//                    StartPush = true;
//                }
//            }
//            else{
//
//                if(StartPush == true) {
//                    buttonPusher.setPosition(.6);
//                    ButtonPusherExtensionStep = 0;
//                    ButtonPusherRetractionStep = -1;
//                    PusherExtend = true;
//                    PusherRetract = false;
//                    EndPush = false;
//                    StartPush = false;
//                }
//
//                if(PusherExtend == true) {
//                    if ((ButtonPusherExtensionStep < 5) && (ButtonPusherExtensionStep > -1) && (ButtonPusherRetractionStep == -1)) {
//
//                        ButtonPusherRetractionStep = -1;
//
//                        for (int i = 0; i < 5; i++) {
//                            sleepOn = true;
//                            timeToSleep = 92;
//
//                            if (sleepOn) {
//
//                                timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                                while (System.currentTimeMillis() < timeToWake) {
//
//                                }
//
//                                timeToSleep = 0;
//
//                                sleepOn = false;
//
//                            }
//
//                        }
//
//                        ButtonPusherExtensionStep = ButtonPusherExtensionStep + 1;
//                    }
//                }
//
//                if((PusherExtend == true) && (ButtonPusherExtensionStep >= 5)) {
//                    buttonPusher.setPosition(.4);
//                    ButtonPusherExtensionStep = -1;
//                    ButtonPusherRetractionStep = 0;
//                    PusherRetract = true;
//                    StartPush = false;
//                    EndPush = false;
//                    PusherExtend = false;
//                }
//
//                if(PusherRetract == true) {
//                    if ((ButtonPusherRetractionStep < 5) && (ButtonPusherRetractionStep > -1) && (ButtonPusherExtensionStep == -1)) {
//
//                        ButtonPusherExtensionStep = -1;
//
//                        for (int i = 0; i < 5; i++) {
//                            sleepOn = true;
//                            timeToSleep = 92;
//
//                            if (sleepOn) {
//
//                                timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                                while (System.currentTimeMillis() < timeToWake) {
//
//                                }
//
//                                timeToSleep = 0;
//
//                                sleepOn = false;
//
//                            }
//                        }
//
//                        ButtonPusherRetractionStep = ButtonPusherRetractionStep + 1;
//
//                    }
//                }
//
//                if((PusherRetract == true) && (ButtonPusherRetractionStep >= 5)) {
//                    buttonInit = false;
//                    push = false;
//                    pushed = true;
//                    ButtonPusherExtensionStep = -1;
//                    ButtonPusherRetractionStep = -1;
//                    StartPush = false;
//                    PusherExtend = false;
//                    PusherRetract = false;
//                    EndPush = true;
//                }
//            }

            /*

            SHOOTING SYSTEM

             */

        if(shoot) {

            if(LauncherM.getCurrentPosition() <= 1000 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.7);
                LauncherM.setPower(0.75);

            }

            else if(LauncherM.getCurrentPosition() <= 1200 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.1);

            }

            else if(LauncherM.getCurrentPosition() <= 2520 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.1);
                LauncherM.setPower(0.85);

            }

            else if (LauncherM.getCurrentPosition() > 2520 + (EncoderClicks - 2520) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                LauncherM.setPower(.1);
            } else {
                LauncherM.setPower(0);
                shoot = false;
                EncoderClicks = EncoderClicks + 2520;
            }
        }
//
//                if(launcherCorrect) {
//
////                    if(LauncherM.getCurrentPosition() > (EncoderClicks - 2400))
////                    {
////
////                        LauncherM.setPower(-0.3);
////
////                    }
//
//                     if (LauncherM.getCurrentPosition() > (EncoderClicks - 2500)) {
//
//                        LauncherM.setPower(-0.03);
//
//                    }
//
//                    else if (LauncherM.getCurrentPosition() < (EncoderClicks - 2540)) {
//
//                        LauncherM.setPower(0.03);
//
//                    }
//
//                    else if ((LauncherM.getCurrentPosition() < (EncoderClicks - 2500)) && (LauncherM.getCurrentPosition() > (EncoderClicks - 2540)))
//                    {
//
//                        LauncherM.setPower(0);
//
//                    }
//                }
//
//
//                if((!launcherCorrect) && (step > 3))
//                {
//
//                    LauncherM.setPower(0);
//
//                }

//            else if(step > 11){
//
//                LauncherM.setPower(0);
//
//            }
    }

    @Override
    public void stop () {

        buttonPusher.setPosition(0.5);

    }
}

