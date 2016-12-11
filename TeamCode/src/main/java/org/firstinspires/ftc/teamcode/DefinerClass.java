package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by aleja on 10/12/2016.
 */

public abstract class DefinerClass extends LinearOpMode{

    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //public DcMotor LiftL;
    //public DcMotor LiftR;

    //public DcMotor LauncherM;
    //public Servo Reloader;

    //public Servo BallG1;
    //public Servo BallG2;

    public DcMotor Roller;

    public OpticalDistanceSensor OD;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    public double x;

    public void initializeRobot(){
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        //LiftL = hardwareMap.dcMotor.get("LiftL");
        //LiftR = hardwareMap.dcMotor.get("LiftR");

        //LauncherM = hardwareMap.dcMotor.get("Launcher");
        //Reloader = hardwareMap.servo.get("Reloader");

        //BallG1 = hardwareMap.servo.get("BallG1");
        //BallG2 = hardwareMap.servo.get("BallG2");

        Roller = hardwareMap.dcMotor.get("Roller");

        //OD = hardwareMap.opticalDistanceSensor.get("OD");
        //encoders for drive train 2 for lift

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
        //LiftR.setDirection(DcMotor.Direction.REVERSE);

        //LiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LiftL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //LiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LiftR.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void runOpMode(double num, boolean L, boolean R) throws InterruptedException{
        initializeRobot();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
            x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (x < 0) {
                x = x + 360;
            }
            telemetry.addData("x", x);
            MoveToAngle(num);
            //LaunchBall(L);
            //Reload(R);
        }
    }
    /* OBSOLETE
    void LaunchBall (boolean Launch) {
        double EncoderClicks = 0;
        if(!Launch){
            return;
        }
        else if(Launch){
            if(LauncherM.getCurrentPosition() < EncoderClicks) {
                LauncherM.setPower(1);
            }
            else{
                LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
                LauncherM.setPower(0);
                return;
            }
        }
    }
    void LaunchSequence (boolean Launch){
        double EncoderClicks = 0;
        boolean reled = false;
        boolean fire = false;
        if(!Launch){
            return;
        }
        if(Launch){
            if (LauncherM.getCurrentPosition() < EncoderClicks / 2) {
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
                    LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
                    return;
                }
            }
        }
    }
    void Reload (boolean Rel) {
        boolean hasReled = false;
        if (!Rel) {
            return;
        }
        if (Rel) {
            if (!hasReled) {
                Reloader.setPosition(255);
                hasReled = true;
            } else {
                Reloader.setPosition(0);
                hasReled = false;
                return;
            }
        }
    }
    */
    void MoveToAngle (double ang){

        //turning clockwise subtracts values
        //turning counter-clockwise adds values
        if (x > ang - 10 && x < ang + 10) {
            //has reached angle therefore end loop
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            return;
        } else if (x < 10 + ang) {
            //turn clockwise
            FR.setPower(-.5);
            FL.setPower(0);
            BR.setPower(-.5);
            BL.setPower(0);
        } else if (x > ang - 10) {
            //turn counter-clockwise
            FR.setPower(0);
            FL.setPower(-.5);
            BR.setPower(0);
            BL.setPower(-.5);
        }
    }
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
