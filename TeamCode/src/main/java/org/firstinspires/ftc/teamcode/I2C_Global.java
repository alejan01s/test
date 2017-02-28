package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;

/*

I2C Sensor Driver

Developed by FTC #8521, 4-H Hard-Hitting Hardware Hooligans on 02/15/2017

Implement this code to access the two color sensors and two sonar sensors mounted to the sides of Bert

Call the Variables Listed Below to Access the Sensors

Color Sensor Variables:

All Variables are Doubles

CRNumber - Get Current Color Number from the Right Color Sensor
CLNumber - Get Current Color Number from the Left Color Sensor

CRRed - Get Current Red Value from the Right Color Sensor
CLRed - Get Current Red Value from the Left Color Sensor

CRGreen - Get Current Green Value from the Right Color Sensor
CLGreen - Get Current Green Value from the Left Color Sensor

CRBlue - Get Current Blue Value from the Right Color Sensor
CLBlue - Get Current Blue Value from the Left color Sensor

CRWhite - Get Current White Value from the Right Color Sensor
CLWhite - Get Current White Value from the Left Color Sensor



Sonar Sensor Variables:

All Variables are Doubles

RightDistanceTime - Get Most Recent MicroSecond Distance for the Right Sonar Sensor
LeftDistanceTime - Get Most Recent MicroSecond Distance for the Left Sonar Sensor

RightIN - Get Most Recent Inch Distance for the Right Sonar Sensor
LeftIN - Get Most Recent Inch Distance for the Left Sonar Sensor

RightCM - Get Most Recent Centimeter Distance for the Right Sonar Sensor
LeftCM - Get Most Recent Centimeter Distance for the Left Sonar Sensor

*/


@TeleOp(name = "I2C_Global", group = "Linear Opmode")
public class I2C_Global extends OpMode {

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

    @Override
    public void init(){

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
        SonarRReader.write8(1, 14);//Gain
        SonarRReader.write8(2, 18);//Range

        //Left Sonar Sensor Configuration
        SonarLReader.write8(1, 14);//Gain
        SonarLReader.write8(2, 18);//Range



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

    }

    @Override
    public void start()
    {



    }

    @Override
    public void loop() {

        //Reading the Color Sensors

        //Obtaining the Color Number
        ColorRNumber = ColorRReader.read(0x04, 1);
        ColorLNumber = ColorLReader.read(0x04, 1);

        CRNumber = ColorRNumber[0] & 0xFF;
        CLNumber = ColorLNumber[0] & 0xFF;

        //Obtaining the Red Value
        ColorRRed = ColorRReader.read(0x05, 1);
        ColorLRed = ColorLReader.read(0x05, 1);

        CRRed = ColorRRed[0] & 0xFF;
        CLRed = ColorLRed[0] & 0xFF;

        //Obtaining the Green Value
        ColorRGreen = ColorRReader.read(0x06, 1);
        ColorLGreen = ColorLReader.read(0x06, 1);

        CRGreen = ColorRGreen[0] & 0xFF;
        CLGreen = ColorLGreen[0] & 0xFF;

        //Obtaining the Blue Value
        ColorRBlue = ColorRReader.read(0x07, 1);
        ColorLBlue = ColorLReader.read(0x07, 1);

        CRBlue = ColorRBlue[0] & 0xFF;
        CLBlue = ColorLBlue[0] & 0xFF;

        //Obtaining the White Value
        ColorRWhite = ColorRReader.read(0x08, 1);
        ColorLWhite = ColorLReader.read(0x08, 1);

        CRWhite = ColorRWhite[0] & 0xFF;
        CLWhite = ColorLWhite[0] & 0xFF;


        //Reading the Sonar Sensors

        //Reading Distance in MicroSeconds
        if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
        {

            //Command the Sonars to Take a Snapshot
            SonarRReader.write8(0, 82);
            SonarLReader.write8(0, 82);

            //Put the Robot to Sleep for 75ms to Allow Sonars to Finish
            Sleep = 75;

            if (SleepEnable == 1) {

                WakeUpTime = System.currentTimeMillis() + Sleep;

                while (System.currentTimeMillis() < WakeUpTime) {

                }

                Sleep = 0;

                SleepEnable = 2;

            }

            //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
            RightDistanceTimeH = SonarRReader.read(0x02, 1);
            RightDistanceTimeL = SonarRReader.read(0x03, 1);

            //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
            LeftDistanceTimeH = SonarLReader.read(0x02, 1);
            LeftDistanceTimeL = SonarLReader.read(0x03, 1);

            //Save Full Distance Values from Last Snapshot
            RightDistanceTime = ((RightDistanceTimeH[0] & 0xFF) * 256) + (RightDistanceTimeL[0] & 0xFF);
            LeftDistanceTime = ((LeftDistanceTimeH[0] & 0xFF) * 256) + (LeftDistanceTimeL[0] & 0xFF);

            //Print These Values to the Screen
            telemetry.addData("Right Distance MicroSeconds: ", RightDistanceTime);
            telemetry.addData("Left Distance MicroSeconds: ", LeftDistanceTime);

            //Turn Off Distance in MicroSeconds Method
            CollectDistanceTime = false;

        }

        //Reading Distance in Inches
        if (CollectDistanceIN == true)//Once the CollectDistanceIN boolean is switched on, the robot takes a snapshot of the distance in Inches
        {

            //Command the Sonars to Take a Snapshot
            SonarRReader.write8(0, 80);
            SonarLReader.write8(0, 80);

            //Put the Robot to Sleep for 75ms to Allow Sonars to Finish
            Sleep = 75;

            if (SleepEnable == 1) {

                WakeUpTime = System.currentTimeMillis() + Sleep;

                while (System.currentTimeMillis() < WakeUpTime) {

                }

                Sleep = 0;

                SleepEnable = 2;

            }

            //Save Values
            RightDistanceIN = SonarRReader.read(0x03, 1);
            LeftDistanceIN = SonarLReader.read(0x03, 1);

            RightIN = RightDistanceIN[0] & 0xFF;
            LeftIN = LeftDistanceIN[0] & 0xFF;

            //Print These Values to the Screen
            telemetry.addData("Right Distance IN: ", RightIN);
            telemetry.addData("Left Distance IN: ", LeftIN);

            //Turn Off Distance in Inches Method
            CollectDistanceIN = false;

        }

        //Reading Distance in CM
        if (CollectDistanceCM == true)//Once the CollectDistanceCM boolean is switched on, the robot takes a snapshot of the distance in Centimeters
        {

            //Command the Sonars to Take a Snapshot
            SonarRReader.write8(0, 81);
            SonarLReader.write8(0, 81);

            //Put the Robot to Sleep for 75ms to Allow Sonars to Finish
            Sleep = 75;

            if (SleepEnable == 1) {

                WakeUpTime = System.currentTimeMillis() + Sleep;

                while (System.currentTimeMillis() < WakeUpTime) {

                }

                Sleep = 0;

                SleepEnable = 2;

            }

            //Save Values
            RightDistanceCM = SonarRReader.read(0x03, 1);
            LeftDistanceCM = SonarLReader.read(0x03, 1);

            RightCM = RightDistanceCM[0] & 0xFF;
            LeftCM = LeftDistanceCM[0] & 0xFF;

            //Print These Values to the Screen
            telemetry.addData("Right Distance CM: ", RightCM);
            telemetry.addData("Left Distance CM: ", LeftCM);

            //Turn Off Distance in Inches Method
            CollectDistanceCM = false;

        }

        //Enable Sleep Method
        if ((CollectDistanceTime || CollectDistanceIN || CollectDistanceCM) && SleepEnable == 2) {

            SleepEnable = 1;

        }

        //Ensuring that Only One Distance Method Runs at a Time
        if (CollectDistanceTime) {

            CollectDistanceIN = false;
            CollectDistanceCM = false;

        }

        if (CollectDistanceIN) {

            CollectDistanceTime = false;
            CollectDistanceCM = false;

        }

        if (CollectDistanceCM) {

            CollectDistanceTime = false;
            CollectDistanceIN = false;

        }


        if (gamepad1.x) {

            CollectDistanceTime = true;

        }

        if (gamepad1.a) {

            CollectDistanceIN = true;

        }

        if (gamepad1.b) {

            CollectDistanceCM = true;

        }

        if (gamepad1.start && !CollectDistanceTime && !CollectDistanceIN && !CollectDistanceCM)
        {

            ColorReadGamepadToggle = true;

        }

        if (gamepad1.back)
        {

            ColorReadGamepadToggle = false;

        }

        if(ColorReadGamepadToggle == true)
        {

            telemetry.addData("Right Color#: ", CRNumber);
            telemetry.addData("Left Color#: ", CLNumber);

            telemetry.addData("Right Color Red: ", CRRed);
            telemetry.addData("Left Color Red: ", CLRed);

            telemetry.addData("Right Color Green: ", CRGreen);
            telemetry.addData("Left Color Green: ", CLGreen);

            telemetry.addData("Right Color Blue: ", CRBlue);
            telemetry.addData("Left Color Blue: ", CLBlue);

            telemetry.addData("Right Color White: ", CRWhite);
            telemetry.addData("Left Color White: ", CLWhite);

        }

    }

    @Override
    public void stop(){

    }


}