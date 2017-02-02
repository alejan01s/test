package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by aleja on 12/26/2016.
 */


@TeleOp(name = "ColorSensorTester", group = "Linear Opmode")
public class ColorTester extends LinearOpMode {

    public ColorSensor colorReader;

    public OpticalDistanceSensor bottomOD;
    public OpticalDistanceSensor frontOD;
    public OpticalDistanceSensor backOD;

    public UltrasonicSensor distanceToWall;

    public boolean isRed;
    public boolean isBlue;

    public void initializeRobot(){
        colorReader = hardwareMap.colorSensor.get("colorSensor");
        bottomOD = hardwareMap.opticalDistanceSensor.get("bottomOD");
        frontOD = hardwareMap.opticalDistanceSensor.get("frontOD");
        backOD = hardwareMap.opticalDistanceSensor.get("backOD");
        distanceToWall = hardwareMap.ultrasonicSensor.get("wallDistance");
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initializeRobot();

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Red: ", colorReader.red());
            telemetry.addData("Blue: ", colorReader.blue());
            telemetry.addData("Green: ", colorReader.green());
            telemetry.addData("Alpha: ", colorReader.alpha());
            telemetry.addData("RGBA: ", colorReader.argb());

            telemetry.addData("bottomOD: ", bottomOD.getRawLightDetected());
            telemetry.addData("frontOD: ", frontOD.getRawLightDetected());
            telemetry.addData("colorOD: ", backOD.getRawLightDetected());

            telemetry.addData("UltraSonicDistance: ", distanceToWall.getUltrasonicLevel());

            telemetry.addData("isRed: ", isRed);
            telemetry.addData("isBlue: ", isBlue);

            telemetry.update();

            isRed = colorReader.red() > 1 && colorReader.red() > colorReader.blue() ? true : false;
            isBlue = colorReader.blue() > 1 && colorReader.blue() > colorReader.red() ? true : false;

            colorReader.enableLed(false);
            bottomOD.enableLed(true);

            if(gamepad1.a){
                frontOD.enableLed(true);
                backOD.enableLed(true);
            }
            else{
                frontOD.enableLed(false);
                backOD.enableLed(false);
            }

            if(isRed) {
                redProcedure();
            }

            if(isBlue){
                blueProcedure();
            }

            idle();
        }
    }

    public void redProcedure () {
        //RUN BUTTON PUSHER AND POSITION FOR RED
    }

    public void blueProcedure () {
        //RUN BUTTON PUSHER AND POSITION FOR BLUE
    }
}
