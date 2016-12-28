package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by aleja on 10/9/2016.
 */
@Disabled
@Autonomous(name="testForFunction", group = "Sensor")
public class angleReadTest extends imuAngleReads {

    @Override
    public void initializeRobot(){
        super.initializeRobot();
    }
    @Override
    public void runOpMode() throws InterruptedException{
        initializeRobot();
        waitForStart();
        super.runOpMode(270);
        while(opModeIsActive()){

            telemetry.addData("x", x);
            telemetry.update();
            idle();
        }
    }
    /*void MoveToAngle (double ang){
        DcMotor BL;
        DcMotor BR;
        DcMotor FR;
        DcMotor FL;

        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

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

        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        x = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        if (x < 0) {
            x = x + 360;
        }

        //turning clockwise subtracts values
        //turning counter-clockwise adds values
        if (x > ang - 50 && x < ang + 50) {
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            return;
        } else if (x > 50 + ang) {
            //turn clockwise
            FR.setPower(.2);
            //has reached angle therefore end loop
            FL.setPower(0);
            BR.setPower(.2);
            BL.setPower(0);
        } else if (x < ang - 50) {
            //turn counter-clockwise
            FR.setPower(0);
            FL.setPower(.2);
            BR.setPower(0);
            BL.setPower(.2);
        }
    }
    */
}
