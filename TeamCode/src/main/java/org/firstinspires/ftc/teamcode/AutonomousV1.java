package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.concurrent.TimeUnit;

/**
 * Created by aleja on 12/4/2016.
 */

//Plan: Launch ball into goal, reload, launch second, then knock ball out of center goal and end.

@Autonomous (name = "AutonomousV1", group = "Sensor")
public class AutonomousV1 extends DefinerClass {

    public void initializeRobot () {
        super.initializeRobot();
    }

    public void runOpMode() throws InterruptedException{
        initializeRobot();
        /*
        *******************
        *                 *
        *  PLACE HOLDER   *
        *     VALUES      *
        *                 *
        *******************
         */
        int step = 0;
        int Angle1 = 270;
        int Angle2 = 90;
        int Angle3 = 180;
        int FinalAngle = 90;
        int NumberOfRevs1 = 10;
        int NumberOfRevs2 = 5;
        int NumberOfRevsFinal = 5;
        waitForStart();
        while(opModeIsActive()){
            //Step 1: Position Robot
            if(step == 0) {
                super.runOpMode(Angle1, false, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            if(step == 1){
                if(FR.getCurrentPosition() < NumberOfRevs1) {
                    BL.setPower(.5);
                    BR.setPower(.5);
                    FR.setPower(.5);
                    FL.setPower(.5);
                }
                else{
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step++;
                }
            }
            if(step == 2){
                super.runOpMode(Angle2, false, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            //Launch ball
            if(step == 3){
                super.runOpMode(Angle2, true, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            //Reload ball
            if(step == 4){
                super.runOpMode(Angle2, false, true);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            //Launch second ball
            if(step == 5){
                super.runOpMode(Angle2, true, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            if(step == 6){
                super.runOpMode(Angle3, false, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            if(step == 7){
                if(FR.getCurrentPosition() < NumberOfRevs2){
                    FR.setPower(.5);
                    FL.setPower(.5);
                    BR.setPower(.5);
                    BL.setPower(.5);
                }
                else{
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    step = step++;
                }
            }
            //LAST ORIENTATION
            if(step == 8){
                super.runOpMode(FinalAngle, false, false);
                TimeUnit.SECONDS.sleep(1);
                step = step++;
            }
            //FINAL MOVE
            if(step == 9){
                if(FR.getCurrentPosition() < NumberOfRevsFinal){
                    FR.setPower(.5);
                    FL.setPower(.5);
                    BR.setPower(.5);
                    BL.setPower(.5);
                }
                else{
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    step = step++;
                }
            }

            telemetry.addData("x", x);
            telemetry.update();
            idle();
        }
    }
}
