package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by aleja on 2/1/2017.
 */

@Autonomous(name = "distanceTester", group = "Sensor")
public class distanceTester extends LinearOpMode {

    public OpticalDistanceSensor colorOD;

    public ColorSensor colorReader;

    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    public void initializeRobot(){
        colorOD = hardwareMap.opticalDistanceSensor.get("backOD");

        colorReader = hardwareMap.colorSensor.get("colorSensor");

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initializeRobot();

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("ColorOD: ", colorOD.getRawLightDetected());

            telemetry.addData("Color ARGB: ", colorReader.argb());
            telemetry.addData("Color Alpha: ", colorReader.alpha());

            telemetry.update();

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
            }

            idle();
        }
    }

}
