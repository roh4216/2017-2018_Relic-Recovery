package org.firstinspires.ftc.teamcode;

import android.hardware.usb.UsbDevice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by riseo on 7/7/2018.
 */
//@Disabled
@TeleOp(name = "Pixy2Test",group = "Pixy")
public class Pixy2Test extends LinearOpMode {
UsbDevice pixyCam;
   @Override
    public void runOpMode(){
    String pixData = pixyCam.toString();
    telemetry.addData("Pixy data is ", pixData);
    telemetry.addData("Pixy is ", pixyCam);
   }
}

