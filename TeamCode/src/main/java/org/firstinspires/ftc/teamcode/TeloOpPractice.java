package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by aleja on 7/10/2016.
 */
@Disabled
@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class TeloOpPractice extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor BL;
    public DcMotor BR;
    public DcMotor FR;
    public DcMotor FL;

    public boolean blue;
    public boolean green;
    public boolean red;
    public boolean lightEnabled;
    public boolean BottomLightEnabled;
    public boolean CurrentState;
    public boolean PrevState;
    public boolean CurrentDir;
    public boolean PrevDir;
    public boolean MechanumWheels;
    public boolean reverseDir;

    public String colorname;

    //public ColorSensor sensor;
    //public ColorSensor sensor1;

    public Sensor GYRO_SENSOR;
    public GyroSensor GYRO;

    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    /*
    float hsvValues1[] = {0F,0F,0F};
    final float values1[] = hsvValues1;
    */

    float direction;
    float vertDirection;
    float horizontalDirection;

    public void initializeRobot(){
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");
        /*
        sensor = hardwareMap.colorSensor.get("FSensor");
        sensor1 = hardwareMap.colorSensor.get("BSensor");
        */
        blue = false;
        green = false;
        red = false;
        lightEnabled = false;
        BottomLightEnabled = false;
        CurrentState = false;
        PrevState = false;
        CurrentDir = false;
        PrevDir = false;
        MechanumWheels = true;
        reverseDir = false;

        colorname = "null";

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        direction = 0f;
        vertDirection = 0f;
        horizontalDirection = 0f;
    }

    public double scaleInput(double value) {
        double[] powerval = {0, 0, 0.25, 0.25, 0.5, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0};
        double retVal;
        int index;
        if (value > 0) {
            index = (int) Math.abs(Range.clip(value * 10, 0, 10));
            retVal = powerval[index];
        } else {
            index = (int) Math.abs(Range.clip(value * -10, 0, 10));
            retVal = -powerval[index];
        }
        return retVal;
    }

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initializeRobot();
        waitForStart();
        runtime.reset();
        waitOneFullHardwareCycle();
        //sensor.enableLed(lightEnabled);
        //sensor.enableLed(BottomLightEnabled);
        while(opModeIsActive()) {
            /*
            CurrentState = gamepad1.x;

            if(CurrentState == true && PrevState != CurrentState) {
                PrevState = CurrentState;
                lightEnabled = true;
                BottomLightEnabled = true;

                sensor.enableLed(lightEnabled);
                sensor1.enableLed(BottomLightEnabled);
                Thread.sleep(100);
            }
            else if(CurrentState == true && PrevState == CurrentState) {
                PrevState = false;
                lightEnabled = false;
                BottomLightEnabled = false;

                sensor.enableLed(lightEnabled);
                sensor1.enableLed(BottomLightEnabled);
                Thread.sleep(100);
            }
            if(!MechanumWheels) {
                CurrentDir = gamepad1.a;

                if(CurrentDir == true && PrevDir != CurrentDir) {
                    PrevDir = CurrentDir;
                    reverseDir = true;
                    Thread.sleep(100);
                }
                else if(CurrentDir == true && PrevDir == CurrentDir) {
                    PrevDir = false;
                    reverseDir = false;
                    Thread.sleep(100);
                }
                if(reverseDir) {
                    BL.setPower(gamepad1.left_stick_y);
                    FL.setPower(gamepad1.left_stick_y);
                    BR.setPower(gamepad1.right_stick_y);
                    FR.setPower(gamepad1.right_stick_y);
                }
                else if(!reverseDir){
                    //reversed dir
                    BL.setPower(-gamepad1.left_stick_y);
                    FL.setPower(-gamepad1.left_stick_y);
                    BR.setPower(-gamepad1.right_stick_y);
                    FR.setPower(-gamepad1.right_stick_y);
                }
            }
            */
            if(MechanumWheels) {
                CurrentDir = gamepad1.a;

                if(CurrentDir == true && PrevDir != CurrentDir) {
                    PrevDir = CurrentDir;
                    reverseDir = true;
                    Thread.sleep(5);
                }
                else if(CurrentDir == true && PrevDir == CurrentDir) {
                    PrevDir = false;
                    reverseDir = false;
                    Thread.sleep(5);
                }
                //final working mechanum wheel code
                if(!reverseDir) {
                    double x = 0;
                    double y = 0;
                    double x2 = 0;

                    final double slowMode = 6;
                    final double joystickThreshold = 10;

                    if (Math.abs(100 * gamepad1.left_stick_x) > joystickThreshold) {
                        y = gamepad1.left_stick_x;
                    } else {
                        y = 0;
                    }
                    if (Math.abs(100 * gamepad1.left_stick_y) > joystickThreshold) {
                        x = gamepad1.left_stick_y;
                    } else {
                        x = 0;
                    }
                    if (Math.abs(100 * gamepad1.right_stick_x) > joystickThreshold) {
                        x2 = gamepad1.right_stick_x;
                    } else {
                        x2 = 0;
                    }
                    if(gamepad1.left_bumper){
                        FR.setPower((-y + x2 + x)/slowMode);
                        BR.setPower((y - x2 + x)/slowMode);
                        FL.setPower((y + x2 + x)/slowMode);
                        BL.setPower((-y - x2 + x)/slowMode);
                    }
                    else {
                        FR.setPower(-y + x2 + x);
                        BR.setPower(y - x2 + x);
                        FL.setPower(y + x2 + x);
                        BL.setPower(-y - x2 + x);
                    }
                }
                if(reverseDir){
                    double x = 0;
                    double y = 0;
                    double x2 = 0;

                    final double joystickThreshold = 5;

                    if (Math.abs(100 * gamepad1.left_stick_x) > joystickThreshold) {
                        y = gamepad1.left_stick_x;
                    } else {
                        y = 0;
                    }
                    if (Math.abs(100 * gamepad1.left_stick_y) > joystickThreshold) {
                        x = gamepad1.left_stick_y;
                    } else {
                        x = 0;
                    }
                    if (Math.abs(100 * gamepad1.right_stick_x) > joystickThreshold) {
                        x2 = gamepad1.right_stick_x;
                    } else {
                        x2 = 0;
                    }
                    FR.setPower(y - x2 - x);
                    BR.setPower(-y + x2 - x);
                    FL.setPower(-y - x2 - x);
                    BL.setPower(y + x2 - x);
                }
                /*
                CurrentDir = gamepad1.a;

                if(CurrentDir == true && PrevDir != CurrentDir) {
                    PrevDir = CurrentDir;
                    reverseDir = true;
                    Thread.sleep(5);
                }
                else if(CurrentDir == true && PrevDir == CurrentDir) {
                    PrevDir = false;
                    reverseDir = false;
                    Thread.sleep(5);
                }

                if(!reverseDir) {
                    //possibly working code
                    final double threshold = 20;

                    if (Math.abs(100 * gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold) {
                        //move
                        FR.setPower(((100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        FR.setPower((-(100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        BR.setPower((-(100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        BL.setPower(((100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                    }
                    if (Math.abs(gamepad1.right_stick_x) > threshold) {
                        //rotate
                        FR.setPower((100 * (-gamepad1.right_stick_x)) / 2);
                        FL.setPower((100 * (-gamepad1.right_stick_x)) / 2);
                        BR.setPower((100 * gamepad1.right_stick_x) / 2);
                        BL.setPower((100 * gamepad1.right_stick_x) / 2);
                    }
                }
                if(reverseDir) {
                    //reversed dir
                    final double threshold = 20;

                    if (Math.abs(100 * gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold) {
                        //move
                        FR.setPower(-((100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        FR.setPower(-(-(100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        BR.setPower(-(-(100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                        BL.setPower(-((100 * gamepad1.left_stick_y) - (100 * gamepad1.left_stick_x)) / 2);
                    }
                    if (Math.abs(gamepad1.right_stick_x) > threshold) {
                        //rotate
                        FR.setPower(-(100 * (-gamepad1.right_stick_x)) / 2);
                        FL.setPower(-(100 * (-gamepad1.right_stick_x)) / 2);
                        BR.setPower(-(100 * gamepad1.right_stick_x) / 2);
                        BL.setPower(-(100 * gamepad1.right_stick_x) / 2);
                    }
                }
                */
            }
            /*
            if(sensor.blue()>sensor.green() && sensor.blue()>sensor.red()){
                blue = true;
            }
            else if(sensor.green()>sensor.blue() && sensor.green()>sensor.red()){
                green = true;
            }
            else if(sensor.red()>sensor.blue() && sensor.red()>sensor.green()){
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

            Color.RGBToHSV(sensor.red()*8, sensor.green()*8, sensor.blue()*8, hsvValues);

            telemetry.addData("Gyro Sensor: ", GYRO_SENSOR.TYPE_GYROSCOPE);
            telemetry.addData("Clear: ", sensor.alpha());
            telemetry.addData("Green: ", sensor.green());
            telemetry.addData("Blue: ", sensor.blue());
            telemetry.addData("Red: ", sensor.red());
            telemetry.addData("Hue: ", hsvValues[0]);
            /*
            Color.RGBToHSV(sensor1.red(), sensor1.green(), sensor1.blue(), hsvValues1);
            telemetry.addData("Clear1: ", sensor1.alpha());
            telemetry.addData("Green1: ", sensor1.green());
            telemetry.addData("Blue1: ", sensor1.blue());
            telemetry.addData("Red1: ", sensor1.red());
            telemetry.addData("Hue1: ", hsvValues1[0]);
            */
            telemetry.addData("Color: ", colorname);
            telemetry.update();
            idle();
        }

        /*Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsvValues);
        telemetry.addData("Clear: ", sensor.alpha());
        telemetry.addData("Green: ", sensor.green());
        telemetry.addData("Blue: ", sensor.blue());
        telemetry.addData("Red: ", sensor.red());
        telemetry.addData("Hue: ", hsvValues[0]);
        Color.RGBToHSV(sensor1.red(), sensor1.green(), sensor1.blue(), hsvValues1);
        telemetry.addData("Clear: ", sensor1.alpha());
        telemetry.addData("Green: ", sensor1.green());
        telemetry.addData("Blue: ", sensor1.blue());
        telemetry.addData("Red: ", sensor1.red());
        telemetry.addData("Hue: ", hsvValues1[0]);*/

        waitForNextHardwareCycle();
    }
}