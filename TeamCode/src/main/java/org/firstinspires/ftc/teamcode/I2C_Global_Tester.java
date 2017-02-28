package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by aleja on 2/27/2017.
 */

@TeleOp (name = "globalFunctionTester", group = "Linear Opmode")
public class I2C_Global_Tester extends I2C_Global{

    double rightDisIN;
    double leftDisIN;
    double rightDisCM;
    double leftDisCM;
    double rightDisMS;
    double leftDisMS;

    boolean runCheck = false;

    public void init(){
        super.init();
    }

    public void start (){

    }

    public void loop () {

        if(runCheck){
            Sleep = 100;

            if (runCheck) {

                WakeUpTime = System.currentTimeMillis() + Sleep;

                while (System.currentTimeMillis() < WakeUpTime) {

                }

                Sleep = 0;

                SleepEnable = 2;

                runCheck = false;

            }
        }

        if(!runCheck){
            super.CollectDistanceTime = true;
            super.loop();
            runCheck = true;
        }

        rightDisIN = super.RightIN;
        leftDisIN = super.LeftIN;

        rightDisCM = super.RightCM;
        leftDisCM = super.LeftCM;

        rightDisMS = super.RightDistanceTime;
        leftDisMS = super.LeftDistanceTime;

        telemetry.addData("Left Distance Inches: ", leftDisIN);
        telemetry.addData("Right Distance Inches: ", rightDisIN);

        telemetry.addData("Left Distance CM: ", leftDisCM);
        telemetry.addData("Right Distance CM: ", rightDisCM);

        telemetry.addData("Left Distance MicroSeconds: ", leftDisMS);
        telemetry.addData("Right Distance MicroSeconds: ", rightDisMS);

    }

    public void stop (){

    }

}
