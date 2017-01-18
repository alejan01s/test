package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "telemetryIMU", group = "Demo")
public class imuTelemetryTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        // in this demo, the IMU is named "IMU" in the robot configuration file
        imuTest imu = new imuTest("imu", hardwareMap);

        // wait to see this on the Driver Station before pressing play, to make sure the IMU has been initialized
        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {
            // the next 4 lines show how to retrieve angles from the imu and use them
            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double pitch = angles[1];
            double roll = angles[2];

            double x = yaw;

            if(x < 0){
                x = x + 360;
            }
            // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
            telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.addData("X: ", x);
            telemetry.update();
        }
    }

}