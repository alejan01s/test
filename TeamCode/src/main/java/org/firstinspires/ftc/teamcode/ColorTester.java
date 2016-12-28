package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by aleja on 12/26/2016.
 */

@Autonomous(name = "ColorSensorTester", group = "Sensor")
public class ColorTester extends DefinerClass {

    @Override
    public void initializeRobot(){
        super.initializeRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initializeRobot();

        waitForStart();
        while(opModeIsActive()){

            idle();
        }
    }
}
