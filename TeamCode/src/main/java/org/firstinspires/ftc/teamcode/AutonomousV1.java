package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/4/2016.
 */

//Plan: Launch ball into goal, reload, launch second, then knock ball out of center goal and end.

@Autonomous (name = "autonomousBASIC", group = "Sensor")
public class AutonomousV1 extends DefinerClass {

    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public boolean seclaunch = false;
    public boolean shoot1 = false;
    public boolean fired = false;

    public DcMotor LauncherM;
    public Servo Reloader;

    public Servo BallG1;
    public Servo BallG2;
    public Servo buttonPusher;
    public Servo buttonPusher2;

    public void initializeRobot () {
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");
        buttonPusher = hardwareMap.servo.get("buttonPusher");
        buttonPusher2 = hardwareMap.servo.get("buttonPusher2");

        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        super.initializeRobot();
    }

    public void runOpMode() throws InterruptedException{
        initializeRobot();
        /*
        *******************
        *                 *
        *  PLACE HOLDER   *
        *     VALUES      *
        *                 *
        *******************
         */
        int step = -1;
        int Angle1 = 270;
        int Angle2 = 90;
        int Angle3 = 180;
        int FinalAngle = 55;
        int NumberOfRevs1 = -400;
        int NumberOfRevs2 = -320;
        int NumberOfRevsFinal = -3500;

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("x", super.x);

            telemetry.update();

            BallG1.setPosition(0);
            BallG2.setPosition(1);
            buttonPusher.setPosition(.5);
            buttonPusher2.setPosition(.5);
            //Step 1: Position Robot
            //if(step == 0) {
                //super.runOpMode(Angle1, false, false);
                //TimeUnit.SECONDS.sleep(1);
                //step = step++;
            //}
            if(step == -1){
                sleep(20000);
                step = step + 1;
            }
            if(step == 0){
                if(FR.getCurrentPosition() > NumberOfRevs1) {
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
                    step = step+1;
                }
            }
            //if(step == 2){
                //super.runOpMode(FinalAngle, false, false);
                //TimeUnit.SECONDS.sleep(2);
                //step = step+1;
            //}
            //if(step == 2){
                //super.runOpMode(Angle2, false, false);
                //TimeUnit.SECONDS.sleep(1);
                //step = step+1;
            //}
            //Launch ball
            //if(step == 1){
                //LaunchSequence();
                //TimeUnit.SECONDS.sleep(3);
                //step = step+1;
            //}
            //Reload ball
            //if(step == 4){
                //super.runOpMode(Angle2, false, true);
                //TimeUnit.SECONDS.sleep(1);
                //step = step++;
            //}
            //Launch second ball
            if(step == 1){
                if(!fired) {
                    shoot1 = true;
                }
                if(fired) {
                    step = step + 1;
                }
            }
            //if(step == 5){
                //super.runOpMode(Angle3, false, false);
                //TimeUnit.SECONDS.sleep(2);
                //step = step+1;
            //}
            /*if(step == 2){
                Thread.sleep(4000);
                if(FR.getCurrentPosition() > NumberOfRevs2){
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                    BR.setPower(-.5);
                    BL.setPower(-.5);
                }
                else{
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    step = step+1;
                }
            }*/

            if(step == 2){
                if(!shoot1) {
                    shoot = true;
                    step = step + 1;
                }
            }

            if(step == 3){
                if(!shoot) {
                    if (FR.getCurrentPosition() > NumberOfRevsFinal) {
                        FR.setPower(-.5);
                        FL.setPower(-.5);
                        BR.setPower(-.5);
                        BL.setPower(-.5);
                    } else {
                        FR.setPower(0);
                        FL.setPower(0);
                        BR.setPower(0);
                        BL.setPower(0);
                        step = step + 1;
                    }
                }
            }
            if(shoot1){
                if (LauncherM.getCurrentPosition() <= EncoderClicks) {
                    LauncherM.setPower(0.85);
                }
                else{
                    LauncherM.setPower(0);
                    sleep(500);
                    fired = true;
                    shoot1 = false;
                    EncoderClicks = EncoderClicks + 2520;
                }
            }

//            if(shoot) {
//
//                if (LauncherM.getCurrentPosition() <= 1400 + (EncoderClicks - 2510)) {
//
//                    Reloader.setPosition(0.65);
//                    LauncherM.setPower(0.75);
//
//                } else if (LauncherM.getCurrentPosition() <= 2250 + (EncoderClicks - 2510)) {
//
//                    Reloader.setPosition(0.1);
//                    LauncherM.setPower(1);
//
//                } else if (LauncherM.getCurrentPosition() > 2250 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
//                    LauncherM.setPower(.1);
//                } else {
//                    LauncherM.setPower(0);
//                    shoot = false;
//                    EncoderClicks = EncoderClicks + 2510;
//                }
//            }
//            if(!shoot){
//                if(LauncherM.getCurrentPosition() > (EncoderClicks - 2510))
//                {
//
//                    LauncherM.setPower(-0.07);
//
//                }
//
//                if(LauncherM.getCurrentPosition() < (EncoderClicks - 2510))
//                {
//
//                    LauncherM.setPower(0.07);
//
//                }
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
            //if(step == 5){
                //super.runOpMode(0, false, false);
                //TimeUnit.SECONDS.sleep(2);
                //step = step+1;
            //}
            //FINAL MOVE
            /*if(step == 6){
                if(FR.getCurrentPosition() > NumberOfRevsFinal){
                    FR.setPower(-.5);
                    FL.setPower(-.5);
                    BR.setPower(-.5);
                    BL.setPower(-.5);
                }
                else{
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    step = step+1;
                }
            }*/
            idle();
        }
    }
}
