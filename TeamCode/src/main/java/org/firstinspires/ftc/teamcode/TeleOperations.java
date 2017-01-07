package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aleja on 10/14/2016.
 */

@TeleOp(name="Main TeleOp", group="Linear Opmode")
public class TeleOperations extends LinearOpMode {

    //DRIVE-TRAIN MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //ROLLER
    public DcMotor Roller;

    //LAUNCHER VARIABLES
    public double EncoderClicks = 2515;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public boolean fire = false;

    //ROLLER VARIABLES
    public double rollerState = 0;

    public DcMotor LiftL;
    public DcMotor LiftR;
    public double encodersLift;
    public boolean tensionLift;

    public Servo BallG1;
    public Servo BallG2;

    public boolean waitForDetoggle = false;
    public boolean manualOverrideBallG1;
    public boolean manualOverrideBallG2;

    public double posBallG1 = 0;
    public double posBallG2 = 0;

    public double overridePosL = 0;
    public double overridePosR = 0;

    public boolean runUp = false;
    public boolean runDown = false;

    //LAUNCHER MECHANISM
    public DcMotor LauncherM;
    public Servo Reloader;

    public Servo buttonPusher;
    public boolean buttonPress;
    public boolean buttonInit;

    public void initializeRobot() throws InterruptedException {

        //CONFIGURATION
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        Roller = hardwareMap.dcMotor.get("Roller");

        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

        LiftL = hardwareMap.dcMotor.get("LiftL");
        LiftR = hardwareMap.dcMotor.get("LiftR");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        //RUN USING ENCODERS + RESET THEM ON START + REVERSE
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

        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        buttonPusher = hardwareMap.servo.get("buttonPusher");

        //LiftL.setDirection(DcMotor.Direction.REVERSE);

        LiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //BallG2.setDirection(Servo.Direction.REVERSE);

        //super.initializeRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initializeRobot();

        //VARIABLES FOR TOGGLING
        boolean SwitchState = false;
        boolean MoveLiftUp = false;
        boolean MoveLiftDown = false;

        boolean SwitchS = false;
        boolean MoveBGsBack = false;
        boolean MoveBGsFor = false;

        boolean buttonPress = false;
        boolean buttonInit = false;

        //ROLLER POWER
        double Roll = 0;

        boolean MoveLiftUpL = false;
        boolean MoveLiftUpR = false;
        boolean MoveLiftDownL = false;
        boolean MoveLiftDownR = false;
        boolean MoveLiftUpPos2L = false;
        boolean MoveLiftUpPos2R = false;
        boolean MoveLiftUpPos2 = false;
        boolean manualOverrideLiftUp;
        boolean manualOverrideLiftDown;
        boolean lRanUp = false;
        boolean rRanUp = false;
        boolean lRanDown = false;
        boolean rRanDown = false;
        tensionLift = false;

        waitForStart();
        while(opModeIsActive()){

            /*

            TELEMETRY DATA

             */

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("LiftL: ", LiftL.getCurrentPosition());
            telemetry.addData("LiftR: ", LiftR.getCurrentPosition());

            telemetry.addData("BallG1 Encoder: ", BallG1.getPosition());
            telemetry.addData("BallG2 Encoder: ", BallG2.getPosition());

            telemetry.addData("Roller Status: ", rollerState);

            telemetry.update();

            /*

            CODE FOR LAUNCHING MECHANISM

            */

            //360 DEGREES = 2745 CLICKS; 30 DEGREES HOLD FOR RELOAD; 130 DEGREES BALL FIRES
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
                        LauncherM.setPower(0.03);
                        Reloader.setPosition(256);
                        resume = true;
                        pause = false;
                    }
                }
                if(resume) {
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks-2515) && LauncherM.getCurrentPosition() <= 1250 + (EncoderClicks - 2515)) {
                        LauncherM.setPower(.05);
                    }
                    else if (LauncherM.getCurrentPosition() > 1250 + (EncoderClicks-2515) && LauncherM.getCurrentPosition() <= 2255 + (EncoderClicks - 2515)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0);
                    } else if (LauncherM.getCurrentPosition() > 2255 + (EncoderClicks - 2515) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.08);
                    } else {
                        LauncherM.setPower(0);
                        shoot = false;
                        resume = false;
                        EncoderClicks = EncoderClicks + 2515;
                    }
                }
            }

            /*

            CODE FOR FIRE SEQUENCE ONLY

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

            CODE FOR MANUALLY RESETTING LAUNCHER ENCODERS

             */

            if(gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x){
                LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }

            /*

            MECHANUM WHEEL CODE

            */


            //FINAL WORKING MECHANUM WHEEL CODE

            double x;
            double y;
            double x2;

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
                FR.setPower((y + x2 + x)/slowMode);
                BR.setPower((-y + x2 + x)/slowMode);
                FL.setPower((-y - x2 + x)/slowMode);
                BL.setPower((y - x2 + x)/slowMode);
            }
            else {
                FR.setPower(y + x2 + x);
                BR.setPower(-y + x2 + x);
                FL.setPower(-y - x2 + x);
                BL.setPower(y - x2 + x);
            }

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

            Roller.setPower(Roll);

            /*

            CODE FOR LIFT

            */

            //BUTTON X IS USED TO TOGGLE LIFT UP AND DOWN
            //MANUAL OVERRIDE IS MAPPED TO TRIGGERS

            //boolean CurrentState = gamepad2.x;
            boolean BallGStowed = BallG1.getPosition()>0.07 ? false : true;

            telemetry.addData("BallGStowed: ", BallGStowed);

            manualOverrideLiftUp = gamepad2.dpad_up ? true : false;
            manualOverrideLiftDown = gamepad2.dpad_down ? true : false;

            if(manualOverrideLiftUp){
                overridePosL = LiftL.getCurrentPosition() - 1;
                overridePosR = LiftR.getCurrentPosition() + 1;
                lRanUp = false;
                rRanUp = false;
                runUp = true;
            }
            else{
                overridePosR = LiftR.getCurrentPosition();
                overridePosL = LiftL.getCurrentPosition();
            }
            if(!runUp && manualOverrideLiftDown){
                overridePosL = LiftL.getCurrentPosition() + 1;
                overridePosR = LiftR.getCurrentPosition() - 1;
                lRanDown = false;
                rRanDown = false;
                runDown = true;
            }
            else{
                overridePosL = LiftL.getCurrentPosition();
                overridePosR = LiftR.getCurrentPosition();
            }

            if(runUp){
                if(LiftL.getCurrentPosition() > overridePosL){
                    LiftL.setPower(-.25);
                }
                else{
                    LiftL.setPower(0);
                    lRanUp = true;
                }
                if(LiftR.getCurrentPosition() < overridePosR){
                    LiftR.setPower(.25);
                }
                else{
                    LiftR.setPower(0);
                    rRanUp = true;
                }
                if(lRanUp && rRanUp){
                    runUp = false;
                }
            }
            else if(runDown){
                if(LiftL.getCurrentPosition() < overridePosL){
                    LiftL.setPower(.25);
                }
                else{
                    LiftL.setPower(0);
                    lRanDown = false;
                }
                if(LiftR.getCurrentPosition() > overridePosR){
                    LiftR.setPower(-.25);
                }
                else{
                    LiftR.setPower(0);
                    rRanDown = true;
                }
                if(lRanDown && rRanDown){
                    runDown = false;
                }
            }

            //BALL GRABBERS MUST NOT BE STOWED TO RUN LIFT AS A SAFETY MEASURE
            if(!BallGStowed) {

                /*
                if (CurrentState == true && SwitchState != CurrentState && MoveLiftDown == false) {
                    SwitchState = CurrentState;
                    MoveLiftUp = true;
                    MoveLiftUpL = true;
                    MoveLiftUpR = true;
                    tensionLift = true;
                    encodersLift = LiftL.getCurrentPosition() - 8000;

                    Thread.sleep(5);
                } else if (CurrentState == true && SwitchState == CurrentState && MoveLiftUp == false) {
                    SwitchState = false;
                    MoveLiftDown = true;
                    MoveLiftDownL = true;
                    MoveLiftDownR = true;
                    tensionLift = true;

                    Thread.sleep(5);
                }
                */

                if(gamepad2.x){
                    MoveLiftUp = true;
                    MoveLiftUpL = true;
                    MoveLiftUpR = true;
                    tensionLift = true;
                    encodersLift = LiftL.getCurrentPosition() - 8000;
                }

                if(gamepad2.a){
                    MoveLiftDown = true;
                    MoveLiftDownL = true;
                    MoveLiftDownR = true;
                    tensionLift = true;
                }

                if(gamepad2.y){
                    MoveLiftUpPos2 = true;
                    MoveLiftUpPos2L = true;
                    MoveLiftUpPos2R = true;
                    tensionLift = true;
                }
            }

            //RAISE LIFT
            if(MoveLiftUp){
                if(tensionLift) {
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);
                    tensionLift = false;
                }
                if(!tensionLift) {
                    if (LiftL.getCurrentPosition() > encodersLift) {
                        LiftL.setPower(-1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftUpL = false;
                    }
                    if (LiftR.getCurrentPosition() < 8000) {
                        LiftR.setPower(1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftUpR = false;
                    }
                    if(!MoveLiftUpL && !MoveLiftUpR){
                        MoveLiftUp =false;
                    }
                }
            }
            //LOWER LIFT
            else if(MoveLiftDown){
                if(tensionLift) {
                    if (LiftL.getCurrentPosition() < 0) {
                        LiftL.setPower(1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftDownL = false;
                    }
                    if (LiftR.getCurrentPosition() > 0) {
                        LiftR.setPower(-1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftDownR = false;
                    }
                    if(!MoveLiftDownL && !MoveLiftDownR){
                        tensionLift = false;
                    }
                }
                if(!tensionLift){
                    Thread.sleep(1000);
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1000);
                    MoveLiftDown = false;
                }
            }
            //RAISE LIFT TO POS 2
            else if(MoveLiftUpPos2){
                if(tensionLift){
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);
                    tensionLift = false;
                }
                if(!tensionLift){
                    if (LiftL.getCurrentPosition() > -1500) {
                        LiftL.setPower(-1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftUpPos2L = false;
                    }
                    if (LiftR.getCurrentPosition() < 1500) {
                        LiftR.setPower(1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftUpPos2R = false;
                    }
                    if(!MoveLiftUpPos2L && !MoveLiftUpPos2R){
                        MoveLiftUpPos2 =false;
                    }
                }
            }

            /*

            CODE FOR BALL GRABBER

             */

            //RIGHT BUMPER USED TO TOGGLE BALL GRABBERS IN AND OUT
            //MANUAL OVERRIDE WORKS ON GAMEPAD2'S JOYSTICKS

            boolean isToggled = gamepad2.right_bumper;

            manualOverrideBallG1 = gamepad2.left_stick_y > .25 || gamepad2.left_stick_y < -.25 ? true : false;
            manualOverrideBallG2 = gamepad2.right_stick_y > .25 || gamepad2.right_stick_y < -.25 ? true : false;

            if(manualOverrideBallG1 || manualOverrideBallG2) {
                if(gamepad2.right_stick_y > .25 || gamepad2.left_stick_y > .25) {
                    posBallG1 = BallG1.getPosition() - .001;
                    posBallG2 = BallG2.getPosition() + .001;
                }
                else{
                    posBallG1 = BallG1.getPosition() + .001;
                    posBallG2 = BallG2.getPosition() - .001;
                }
            }
            else{
                posBallG1 = BallG1.getPosition();
                posBallG2 = BallG2.getPosition();
            }

            if(manualOverrideBallG1){
                BallG1.setPosition(posBallG1);
                waitForDetoggle = true;
            }

            if(manualOverrideBallG2){
                BallG2.setPosition(posBallG2);
                waitForDetoggle = true;
            }

            if(isToggled){
                waitForDetoggle = false;
            }

            if(!manualOverrideBallG1 && !manualOverrideBallG2 && !waitForDetoggle) {
                if (isToggled) {
                    rollerState = rollerState != 2 ? rollerState + 1 : 1;
                    Thread.sleep(500);
                }
                if(gamepad2.left_bumper){
                    rollerState = 0;
                }
                //ball grabber 1 reach pos 0 to stow 1 to grab
                //ball grabber 2 reach pos 1 to stow 0 to grab
                if (rollerState == 0) {
                    BallG1.setPosition(0);
                    BallG2.setPosition(1);
                } else if (rollerState == 1) {
                    BallG1.setPosition(0.3);
                    BallG2.setPosition(0.7);
                } else if (rollerState == 2) {
                    BallG1.setPosition(.35);
                    BallG2.setPosition(.65);
                }
            }

            /*

            BUTTON PUSHER

            */

            if(!buttonInit){
                buttonPusher.setPosition(.5);
                if(gamepad1.y){
                    buttonInit = true;
                }
            }
            else{
                buttonPusher.setPosition(.4);
                Thread.sleep(1500);
                buttonPusher.setPosition(.6);
                Thread.sleep(1500);
                buttonInit = false;
            }

        }
    }
}
