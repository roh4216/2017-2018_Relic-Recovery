package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by riseo on 7/7/2018.
 */

public class Pixy2Test extends LinearOpMode {

    /**
     * Created by Sarthak on 7/3/2017.
     */

    I2cDevice pixyCam;
        I2cDeviceSynchImpl pixyCamReader;
        I2cAddr pixyCamAddress = I2cAddr.create8bit(0x01);
       // Servo pan, tilt;

        double x, y, width, height, numObjs;

        byte[] pixyData;

        @Override
        public void runOpMode() throws InterruptedException {

//            pan = hardwareMap.servo.get("pan");
//            tilt = hardwareMap.servo.get("tilt");

            double xServo = 0.5, yServo = 0.5;
//            pan.setPosition(xServo);
//            tilt.setPosition(yServo);

            pixyCam = hardwareMap.i2cDevice.get("pixy");
            pixyCamReader = new I2cDeviceSynchImpl(pixyCam, pixyCamAddress, false);
            pixyCamReader.engage();

            waitForStart();

            while(opModeIsActive()){
                pixyCamReader.engage();
                pixyData = pixyCamReader.read(0x51, 5);
//                pan.setPosition(xServo);
//                tilt.setPosition(yServo);

                x = pixyData[1];
                y = pixyData[2];
                numObjs = pixyData[0];

                if(x < 0 && numObjs > 0){
                    if(xServo < 1 || xServo > 0){
                        xServo += 0.02;
                    }
                }else if(x > 0 && numObjs > 0){
                    if(xServo < 1 || xServo > 0){
                        xServo -= 0.02;
                    }
                }

            /*if(y < 0 && numObjs > 0){
                if(yServo < 1 || yServo > 0){
                    yServo += 0.02;
                }
            }else if(y>0 && numObjs > 0){
                if(yServo < 1 || yServo > 0){
                    yServo -= 0.02;
                }
            }*/

                telemetry.addData("0", pixyData[0]);
                telemetry.addData("1", pixyData[1]);
                telemetry.addData("2", pixyData[2]);
                telemetry.addData("3", pixyData[3]);
                telemetry.addData("4", pixyData[4]);
                telemetry.addData("Length", pixyData.length);
                telemetry.update();
                pixyCam.readI2cCacheFromController();
            }

        }
    }

