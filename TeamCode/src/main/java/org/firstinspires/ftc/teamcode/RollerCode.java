package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by aleja on 10/14/2016.
 */

@TeleOp(name="RollerCode", group = "Linear Opmode")
@Disabled
public class RollerCode extends DefinerClass {

    public void initializeRobot(){
        super.initializeRobot();
    }

    public void runOpMode () throws InterruptedException {
        initializeRobot();
        double Roll = 0;
        waitForStart();
        while(opModeIsActive()){
            if(gamepad2.right_trigger > 0.1f && gamepad2.left_trigger < 0.1f){
                Roll = gamepad2.right_trigger;
            }
            else{
                Roll = 0;
            }
            if(gamepad2.left_trigger > 0.1f && gamepad2.right_trigger < 0.1f){
                Roll = gamepad2.left_trigger;
            }
            else{
                Roll = 0;
            }
            super.Roller.setPower(Roll);
            idle();
        }
    }
}
