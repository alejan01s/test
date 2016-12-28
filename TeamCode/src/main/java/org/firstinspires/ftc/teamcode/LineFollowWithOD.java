package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aleja on 10/12/2016.
 */
@Disabled
@Autonomous(name="Line Follow", group="Sensor")
public class LineFollowWithOD extends DefinerClass{


    public void initializeRobot(){
        super.initializeRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initializeRobot();
        waitForStart();
        while(opModeIsActive()){
            //white is more than .06
            //black is less than .06
            double value = super.OD.getLightDetected();
            String color = "null";
            boolean OnLine = false;
            telemetry.update();
            telemetry.addData("Value", value);
            telemetry.addData("Color", color);

            if(value > .06){
                color = "white";
                OnLine = true;
            }
            else{
                color = "black";
                OnLine = false;
            }
            if(OnLine){
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(0);
                FR.setPower(0);
            }
            else{
                BL.setPower(0);
                FL.setPower(0);
                BR.setPower(.2);
                FR.setPower(.2);
            }
            idle();
        }
    }
}
