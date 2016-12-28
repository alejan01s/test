package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by aleja on 12/26/2016.
 */

@Autonomous(name = "ColorSensorTester", group = "Sensor")
public class ColorTester extends LinearOpMode {

    public ColorSensor colorReader;

    public boolean isRed;
    public boolean isBlue;

    public void initializeRobot(){
        colorReader = hardwareMap.colorSensor.get("color");
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

            telemetry.addData("isRed: ", isRed);
            telemetry.addData("isBlue: ", isBlue);

            telemetry.update();

            isRed = colorReader.red() > 1 && colorReader.red() > colorReader.blue() ? true : false;
            isBlue = colorReader.blue() > 1 && colorReader.blue() > colorReader.red() ? true : false;

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
