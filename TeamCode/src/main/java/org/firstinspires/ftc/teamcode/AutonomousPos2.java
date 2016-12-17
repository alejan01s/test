package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/4/2016.
 */

//Plan: Launch ball into goal, reload, launch second, then knock ball out of center goal and end.

@Autonomous (name = "AutonomousPos2", group = "Sensor")
public class AutonomousPos2 extends DefinerClass {

    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public boolean seclaunch = false;

    public DcMotor LauncherM;
    public Servo Reloader;

    public void initializeRobot () {
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

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
        int step = 0;
        int Angle1 = 20;
        int Angle2 = 90;
        int Angle3 = 180;
        int FinalAngle = 55;
        int NumberOfRevs1 = -125;
        int NumberOfRevs2 = -2900;
        int NumberOfRevsFinal = -2600;
        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("x", x);

            telemetry.update();
            //Step 1: Position Robot
            //if(step == 0) {
            //super.runOpMode(Angle1, false, false);
            //TimeUnit.SECONDS.sleep(1);
            //step = step++;
            //}
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
                shoot = true;
                step = step+1;
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
                if(!shoot) {
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
            /*if(step == 4) {
                super.runOpMode(Angle1, false, false);
                step = step + 1;
            }
            if(step == 5){
                NumberOfRevs2 = FR.getCurrentPosition() - 300;
                step = step + 1;
            }*/
            if(step == 4){
                Thread.sleep(1000);
                step = step + 1;
            }
            if(step == 5){
                if(FR.getCurrentPosition() > NumberOfRevs2){
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
}
