package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by aleja on 7/10/2016.
 */
public class PracticeAutonV2 extends LinearOpMode {
    public DcMotor BL;
    public DcMotor BR;
    public DcMotor FR;
    public DcMotor FL;

    public ColorSensor BColor;
    public ColorSensor FColor;

    public OpticalDistanceSensor ODSensor;

    public boolean blue;
    public boolean green;
    public boolean red;
    public boolean HasPassed;
    public boolean hasSwitched;

    public String colorname;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;

    float hsvValues1[] = {0F, 0F, 0F};
    final float values1[] = hsvValues1;

    public void initializeRobot() {
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        BColor = hardwareMap.colorSensor.get("BSensor");
        FColor = hardwareMap.colorSensor.get("FSensor");

        blue = false;
        green = false;
        red = false;

        ODSensor = hardwareMap.opticalDistanceSensor.get("odsensor");

        HasPassed = false;
        hasSwitched = false;

        colorname = "null";

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        BColor.enableLed(true);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        waitOneFullHardwareCycle();
        while (opModeIsActive()) {
            double value = ODSensor.getLightDetected();
            telemetry.addData("Value", value);

            if(value<.007){
                BL.setPower(-.1);
                BR.setPower(-.1);
                FL.setPower(-.1);
                FR.setPower(-.1);
            }
            else if(value>=.009){
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);
            }
            if(FColor.blue()>FColor.green() && FColor.blue()>FColor.red()){
                blue = true;
            }
            else if(FColor.green()>FColor.blue() && FColor.green()>FColor.red()){
                green = true;
            }
            else if(FColor.red()>FColor.blue() && FColor.red()>FColor.green()){
                red = true;
            }

            if(blue){
                colorname = "blue";
            }
            if(green){
                colorname = "green";
            }
            if(red){
                colorname = "red";
            }
            //sensor1.enableLed(gamepad1.left_bumper);

            Color.RGBToHSV(FColor.red()*8, FColor.green()*8, FColor.blue()*8, hsvValues);

            telemetry.addData("Color: ", colorname);
            telemetry.addData("Clear: ", FColor.alpha());
            telemetry.addData("Green: ", FColor.green());
            telemetry.addData("Blue: ", FColor.blue());
            telemetry.addData("Red: ", FColor.red());
            telemetry.addData("Hue: ", hsvValues[0]);
            /*
            Color.RGBToHSV(BColor.red()*8, BColor.green()*8, BColor.blue()*8, hsvValues);
            telemetry.addData("White", BColor.alpha());
            telemetry.addData("Hue: ", hsvValues[0]);
                /*
                //right turn
                if(BColor.alpha()<=30)
                {
                    BL.setPower(-.4);
                    FL.setPower(-.4);
                    BR.setPower(0.15);
                    BL.setPower(0.15);
                }
                else if(BColor.alpha()>=31 && BColor.alpha()<=49)
                {
                    BL.setPower(-.2);
                    FL.setPower(-.2);
                    BR.setPower(-.2);
                    FR.setPower(-.2);
                }
                //left turn
                else if(BColor.alpha()>=50)
                {
                    BL.setPower(0.15);
                    FL.setPower(0.15);
                    BR.setPower(-.4);
                    FR.setPower(-.4);
                    //csleep(1000);
                }
                */
            //right turn
            /*
            if(BColor.alpha()<=30 && HasPassed == false)
            {
                BL.setPower(-.4);
                FL.setPower(-.4);
                BR.setPower(0.15);
                FR.setPower(0.15);
                hasSwitched = false;
            }
            if(BColor.alpha()<=30 && HasPassed == true)
            {
                BL.setPower(0.15);
                FL.setPower(0.15);
                BR.setPower(-.4);
                FR.setPower(-.4);
                hasSwitched = true;
            }
            else if(BColor.alpha()>=50)
            {
                if(!HasPassed && !hasSwitched)
                {
                    HasPassed = true;
                }
                if(HasPassed && hasSwitched)
                {
                    HasPassed = false;
                }
                BL.setPower(-.2);
                FL.setPower(-.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
            }
        }
        */

            waitForNextHardwareCycle();
        }
    }
}