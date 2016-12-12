package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aleja on 10/14/2016.
 */

@TeleOp(name="Main TeleOp", group="Linear Opmode")
public class TeleOperations extends DefinerClass {

    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public boolean fire = false;

    public DcMotor LauncherM;
    public Servo Reloader;

    @Override
    public void initializeRobot() {
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.initializeRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initializeRobot();

        boolean PrevDir = false;
        boolean reverseDir = false;

        boolean SwitchState = false;
        boolean MoveLiftUp = false;
        boolean MoveLiftDown = false;

        boolean SwitchS = false;
        boolean MoveBGsBack = false;
        boolean MoveBGsFor = false;

        double Roll = 0;

        waitForStart();
        while(opModeIsActive()){
            //UPDATE TELEMETRY
            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());
            telemetry.addData("FL: ", super.FL.getCurrentPosition());
            telemetry.addData("FR: ", super.FR.getCurrentPosition());
            telemetry.addData("BL: ", super.BL.getCurrentPosition());
            telemetry.addData("BR: ", super.BR.getCurrentPosition());

            telemetry.update();

            /*

            CODE FOR LAUNCHING MECHANISM

            */
            //360 = 2745, 30 degrees for shooting, 130 degrees
            if(gamepad1.right_trigger > .75){
                shoot = true;
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
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks-2510) && LauncherM.getCurrentPosition() <= 1250 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(.05);
                    }
                    else if (LauncherM.getCurrentPosition() > 1250 + (EncoderClicks-2510) && LauncherM.getCurrentPosition() <= 1800 + (EncoderClicks - 2510)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0);
                    } else if (LauncherM.getCurrentPosition() > 1800 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.08);
                    } else {
                        LauncherM.setPower(0);
                        shoot = false;
                        resume = false;
                        EncoderClicks = EncoderClicks + 2510;
                    }
                }
            }

            /*

            CODE FOR FIRE ONLY

             */

            if(gamepad1.right_bumper){
                fire = true;
            }

            if(fire){
                if(LauncherM.getCurrentPosition() <= EncoderClicks){
                    LauncherM.setPower(1);
                }
                else{
                    LauncherM.setPower(0);
                    fire = false;
                    EncoderClicks = EncoderClicks + 2510;
                }
            }
            /*

            CODE FOR RESETING ENCODERS (LAUNCHER)

             */

            if(gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x){
                LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }

            /*

            MECHANUM WHEEL CODE + TOGGLEABLE REVERSE

            */
            /*
            boolean CurrentDir = gamepad1.a;

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
            */
            //final working mechanum wheel code
            //REVERSE DIRECTION IS OBSOLETE / NO LONGER NEEDED
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
                    super.FR.setPower((y + x2 + x)/slowMode);
                    super.BR.setPower((-y + x2 + x)/slowMode);
                    super.FL.setPower((-y - x2 + x)/slowMode);
                    super.BL.setPower((y - x2 + x)/slowMode);
                }
                else {
                    super.FR.setPower(y + x2 + x);
                    super.BR.setPower(-y + x2 + x);
                    super.FL.setPower(-y - x2 + x);
                    super.BL.setPower(y - x2 + x);
                }
            }
            /*if(reverseDir){
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
                super.FR.setPower(y - x2 - x);
                super.BR.setPower(-y + x2 - x);
                super.FL.setPower(-y - x2 - x);
                super.BL.setPower(y + x2 - x);
            }
            */
            /*

            CODE FOR ROLLERS

            */

            if(gamepad2.right_trigger > 0.1f && gamepad2.left_trigger < 0.1f){
                Roll = gamepad2.right_trigger;
            }
            else if(gamepad2.left_trigger > 0.1f && gamepad2.right_trigger < 0.1f){
                Roll = -gamepad2.left_trigger;
            }
            else {
                Roll = 0;
            }

            super.Roller.setPower(Roll);

            /*

            CODE FOR LIFT

            */

            //BUTTON X IS USED TO TOGGLE LIFT UP AND DOWN
            /*
            boolean CurrentState = gamepad2.x;

            if(CurrentState == true && SwitchState != CurrentState && MoveLiftDown == false){
                SwitchState = CurrentState;
                MoveLiftUp = true;

                Thread.sleep(5);
            }
            else if(CurrentState == true && SwitchState == CurrentState && MoveLiftUp == false){
                SwitchState = false;
                MoveLiftDown = true;

                Thread.sleep(5);
            }

            //RAISE LIFT
            if(MoveLiftUp){
                if(super.LiftL.getCurrentPosition() < /*HOW EVER MANY CLICKS **CURRENT NUMBER IS PLACEHOLDER***//* 1) {
                    super.LiftL.setPower(1);
                    super.LiftR.setPower(1);
                }
                MoveLiftUp = false;
            }
            //LOWER LIFT
            else if(MoveLiftDown){
                if(super.LiftL.getCurrentPosition() > 0){
                    super.LiftL.setPower(-1);
                    super.LiftR.setPower(-1);
                }
                MoveLiftDown = false;
            }
            */
            /*

            CODE FOR BALL GRABBER

             */

            //RIGHT BUMPER USED TO TOGGLE BALL GRABBERS IN AND OUT
            /*
            boolean CurrentS = gamepad2.right_bumper;

            if(CurrentS == true && SwitchS != CurrentS && MoveBGsBack == false){
                SwitchS = CurrentS;
                MoveBGsFor = true;

                Thread.sleep(5);
            }
            else if(CurrentS == true && SwitchS == CurrentS && MoveBGsFor == false){
                SwitchS = false;
                MoveBGsBack = true;

                Thread.sleep(5);
            }

            if(MoveBGsFor){
                super.BallG1.setPosition(256);
                super.BallG2.setPosition(256);

                MoveBGsFor = false;
            }
            else if(MoveBGsBack){
                super.BallG1.setPosition(0);
                super.BallG2.setPosition(0);

                MoveBGsBack = false;
            }*/
        }
    }
}
