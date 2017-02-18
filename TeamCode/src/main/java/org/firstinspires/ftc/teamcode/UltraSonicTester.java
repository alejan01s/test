package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by aleja on 12/26/2016.
 */


@TeleOp(name = "ultraSonicSensorTester", group = "Linear Opmode")
public class UltraSonicTester extends LinearOpMode {

    I2cDevice UltraSonicL;
    I2cDeviceSynch UltraSonicReaderL;
    byte[] UltraSonicCacheL;

    I2cDevice UltraSonicR;
    I2cDeviceSynch UltraSonicReaderR;
    byte[] UltraSonicCacheR;

    public void initializeRobot(){
        UltraSonicL = hardwareMap.i2cDevice.get("ultraSonicL");
        UltraSonicR = hardwareMap.i2cDevice.get("ultraSonicR");

        UltraSonicReaderL = new I2cDeviceSynchImpl(UltraSonicL, I2cAddr.create8bit(0x4a), false);
        UltraSonicReaderR = new I2cDeviceSynchImpl(UltraSonicR, I2cAddr.create8bit(0x4c), false);

        UltraSonicReaderL.engage();
        UltraSonicReaderR.engage();
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initializeRobot();

        waitForStart();
        while(opModeIsActive()){
            UltraSonicCacheL = UltraSonicReaderL.read(0x52, 1);
            UltraSonicCacheR = UltraSonicReaderR.read(0x52, 1);

            telemetry.addData("Left Distance: ", UltraSonicCacheL[0] & 0xFF);
            telemetry.addData("Right Distance: ", UltraSonicCacheR[0] & 0xFF);
            idle();
        }
    }
}
