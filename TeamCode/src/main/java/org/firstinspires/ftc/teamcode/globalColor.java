package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


/**
 * Created by Trevor A. Babb on 02/15/2017.
 */

@Disabled
@TeleOp(name = "forget", group = "Linear Opmode")
public class globalColor extends OpMode {

    I2cDevice ColorR;
    I2cDevice ColorL;
    I2cDeviceSynch ColorRReader;
    I2cDeviceSynch ColorLReader;
    byte[] ColorRNumber;
    byte[] ColorLNumber;
    byte[] ColorRRed;
    byte[] ColorLRed;
    byte[] ColorRGreen;
    byte[] ColorLGreen;
    byte[] ColorRBlue;
    byte[] ColorLBlue;
    byte[] ColorRWhite;
    byte[] ColorLWhite;


    @Override
    public void init() {


        ColorR = hardwareMap.i2cDevice.get("ColorR");
        ColorL = hardwareMap.i2cDevice.get("ColorL");

        ColorRReader = new I2cDeviceSynchImpl(ColorR, I2cAddr.create8bit(0x3a), false);
        ColorLReader = new I2cDeviceSynchImpl(ColorL, I2cAddr.create8bit(0x3c), false);

        ColorRReader.engage();
        ColorLReader.engage();

    }

    @Override
    public void start() {

        ColorRReader.write8(3, 1);
        ColorLReader.write8(3, 1);

    }

    @Override
    public void loop() {

        ColorRNumber = ColorRReader.read(0x04, 1);
        ColorLNumber = ColorLReader.read(0x04, 1);

        ColorRRed = ColorRReader.read(0x05, 1);
        ColorLRed = ColorLReader.read(0x05, 1);

        ColorRGreen = ColorRReader.read(0x06, 1);
        ColorLGreen = ColorLReader.read(0x06, 1);

        ColorRBlue = ColorRReader.read(0x07, 1);
        ColorLBlue = ColorLReader.read(0x07, 1);

        ColorRWhite = ColorRReader.read(0x08, 1);
        ColorLWhite = ColorLReader.read(0x08, 1);

        telemetry.addData("Right Color#: ", ColorRNumber[0] & 0xFF);
        telemetry.addData("Left Color#: ", ColorLNumber[0] & 0xFF);

        telemetry.addData("Right Color Red: ", ColorRRed[0] & 0xFF);
        telemetry.addData("Left Color Red: ", ColorLRed[0] & 0xFF);

        telemetry.addData("Right Color Green: ", ColorRGreen[0] & 0xFF);
        telemetry.addData("Left Color Green: ", ColorLGreen[0] & 0xFF);

        telemetry.addData("Right Color Blue: ", ColorRBlue[0] & 0xFF);
        telemetry.addData("Left Color Blue: ", ColorLBlue[0] & 0xFF);

        telemetry.addData("Right Color White: ", ColorRWhite[0] & 0xFF);
        telemetry.addData("Left Color White: ", ColorLWhite[0] & 0xFF);

    }

    @Override
    public void stop() {

        ColorRReader.write8(3, 1);
        ColorLReader.write8(3, 1);

    }
}
