package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aleja on 7/10/2016.
 */
public class PracticeAutonV1 extends LinearOpMode{
    public DcMotor BL;
    public DcMotor BR;
    public DcMotor FR;
    public DcMotor FL;

    public ColorSensor BColor;
    public ColorSensor FColor;

    public boolean HasPassed;
    public boolean hasSwitched;

    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;

    float hsvValues1[] = {0F,0F,0F};
    final float values1[] = hsvValues1;

    public void initializeRobot(){
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        BColor = hardwareMap.colorSensor.get("BSensor");
        FColor = hardwareMap.colorSensor.get("FSensor");

        HasPassed = false;
        hasSwitched = false;

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        BColor.enableLed(true);

    }

    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        waitOneFullHardwareCycle();
        while(opModeIsActive()) {

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
                    //sleep(1000);
                }
                */
            //right turn
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

            if(gamepad1.x)
            {

                StopRobot();

            }
        }

        waitForNextHardwareCycle();
    }

    void StopRobot () {
        BL.setPower(-.1);
        FL.setPower(-.1);
        BR.setPower(-.1);
        BL.setPower(-.1);

        BL.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
}