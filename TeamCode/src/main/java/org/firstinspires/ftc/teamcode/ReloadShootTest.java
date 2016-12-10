package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/5/2016.
 */

@TeleOp(name = "Reload and Shoot Practice", group = "Linear Opmode")
public class ReloadShootTest extends LinearOpMode {

    public DcMotor LauncherM;
    public Servo Reloader;

    public double EncoderClicks = 2500;
    public boolean shoot = false;
    public boolean reled = false;
    public boolean fire = false;
    public boolean pause = false;
    public boolean resume = false;

    public void initializeRobot(){
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode () throws InterruptedException{
        initializeRobot();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());
            telemetry.update();
            //One Revolution is 2745
            /*

            OBSOLETE WORK WITH BUTTON A

             */
            /*
            if(gamepad1.x){
                if (LauncherM.getCurrentPosition() <= 825) {
                    LauncherM.setPower(1);
                }
                else {
                    LauncherM.setPower(0);
                    if(!reled){
                        Reloader.setPosition(255);
                        reled = true;
                    }
                    else if(reled){
                        Reloader.setPosition(0);
                        fire = true;
                    }
                }
                if(fire){
                    if(LauncherM.getCurrentPosition() < EncoderClicks){
                        LauncherM.setPower(1);
                    }
                    else{
                        LauncherM.setPower(0);
                        reled = false;
                        LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
                    }
                }
            }
            */
            if(gamepad1.a){
                shoot = true;
            }

            if(shoot) {
                if(!resume) {
                    if (LauncherM.getCurrentPosition() <= 700 + (EncoderClicks - 2500)) {
                        LauncherM.setPower(.05);
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
                if(resume) {
                    if (LauncherM.getCurrentPosition() > 700 + (EncoderClicks-2500) && LauncherM.getCurrentPosition() <= 2000 + (EncoderClicks - 2500)) {
                        LauncherM.setPower(1);
                    } else if (LauncherM.getCurrentPosition() > 2000 + (EncoderClicks - 2500) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.1);
                        Reloader.setPosition(0);
                    } else {
                        LauncherM.setPower(0);
                        shoot = false;
                        resume = false;
                        EncoderClicks = EncoderClicks + 2500;
                    }
                }
            }
        }
    }
}
