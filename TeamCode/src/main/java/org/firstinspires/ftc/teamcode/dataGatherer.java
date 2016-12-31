package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/4/2016.
 */

@TeleOp(name = "dataGatherer", group = "Linear Opmode")
public class dataGatherer extends LinearOpMode {


    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;


    //VARIABLES FOR LAUNCHER
    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;


    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double x;
    public boolean turnCompleted = false;

    /*

    ARRAY FOR LOGGING INFO

     */

    double[] log;
    public int i = 0;
    public boolean increment = false;
    //1: MOVE TO SHOOT, SHOOT 2: TURN TO LINE 3: MOVE TO LINE, RUN LINE FOLLOW, PUSH BUTTON 4: BACK UP 5: TURN TO NEXT LINE 6: MOVE TO LINE, LINE FOLLOW, PUSH BUTTON 7: TURN TO BALL 8: MOVE TO BALL
    //1: ENCODERS 2: ORIENTATION 3: ENCODERS 4: ENCODERS 5: ORIENTATION 6: ENCODERS 7: ORIENTATION 8: ENCODERS

    public void initializeRobot () {


        //DRIVE-TRAIN MOTORS
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");
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


        //LAUNCHER MOTORS
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");
        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void runOpMode() throws InterruptedException{

        initializeRobot();


        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //SEQUENCE VARIABLE
        int step = 0;

        //REVOLUTION VARIABLES
        int NumberOfRevs1 = -300;
        int NumberOfRevs2 = -3500;

        //ANGLE VARIABLES
        double Angle1 = 50;

        composeTelemetry();
        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("x", x);


            telemetry.update();

            //ESTABLISH ROTATION
            x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (x < 0) {
                x = x + 360;
            }

            //SEQUENCES
            double x1 = 0;
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
                x1 = gamepad1.left_stick_y;
            } else {
                x1 = 0;
            }
            if (Math.abs(100 * gamepad1.right_stick_x) > joystickThreshold) {
                x2 = gamepad1.right_stick_x;
            } else {
                x2 = 0;
            }
            if(gamepad1.left_bumper){
                FR.setPower((y + x2 + x1)/slowMode);
                BR.setPower((-y + x2 + x1)/slowMode);
                FL.setPower((-y - x2 + x1)/slowMode);
                BL.setPower((y - x2 + x1)/slowMode);
            }
            /*

            SHOOTING SYSTEM

             */
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
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks-2515) && LauncherM.getCurrentPosition() <= 1250 + (EncoderClicks - 2515)) {
                        LauncherM.setPower(.05);
                    }
                    else if (LauncherM.getCurrentPosition() > 1250 + (EncoderClicks-2515) && LauncherM.getCurrentPosition() <= 1800 + (EncoderClicks - 2515)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0);
                    } else if (LauncherM.getCurrentPosition() > 1800 + (EncoderClicks - 2515) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.08);
                    } else {
                        LauncherM.setPower(0);
                        shoot = false;
                        resume = false;
                        EncoderClicks = EncoderClicks + 2515;
                    }
                }
            }

            if(gamepad1.a){
                increment = true;
                Thread.sleep(500);
            }

            if(increment){
                i++;
                increment = false;
            }
            //1: ENCODERS 2: ORIENTATION 3: ENCODERS 4: ENCODERS 5: ORIENTATION 6: ENCODERS 7: ORIENTATION 8: ENCODERS
            if(i == 1){
                log[0] = FL.getCurrentPosition();
            }
            if(i == 2){
                log[1] = x;
            }
            if(i == 3){
                log[2] = FL.getCurrentPosition();
            }
            if(i == 4){
                log[3] = FL.getCurrentPosition();
            }
            if(i == 5){
                log[4] = x;
            }
            if(i == 6){
                log[5] = FL.getCurrentPosition();
            }
            if(i == 7){
                log[6] = x;
            }
            if(i == 8){
                log[7] = FL.getCurrentPosition();
            }

            telemetry.addData("LOG DATA 1: ", log[0]);
            telemetry.addData("LOG DATA 2: ", log[1]);
            telemetry.addData("LOG DATA 3: ", log[2]);
            telemetry.addData("LOG DATA 4: ", log[3]);
            telemetry.addData("LOG DATA 5: ", log[4]);
            telemetry.addData("LOG DATA 6: ", log[5]);
            telemetry.addData("LOG DATA 7: ", log[6]);
            telemetry.addData("LOG DATA 8: ", log[7]);

            idle();
            }
        }


    //IMU
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("x", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
