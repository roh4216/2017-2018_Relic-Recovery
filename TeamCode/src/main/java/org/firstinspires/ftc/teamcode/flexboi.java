package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * Created by riseo on 3/26/2018.
 */
@TeleOp(name="flexboi", group="OpMode")
@Disabled
public class flexboi extends OpMode {

    AnalogInput flex;

    public void init(){

       // flex = hardwareMap.analogInput.get("flex");

    }
    public void loop(){

      //  telemetry.addData("flex", flex.getVoltage());
        telemetry.update();
    }
    public void stop(){

    }


}
