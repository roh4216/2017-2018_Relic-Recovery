package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by riseo on 3/26/2018.
 */
@TeleOp(name="BeamBreak", group="OpMode")
public class BeanBreak extends OpMode {

DigitalChannel beam;

public void init(){

    beam = hardwareMap.digitalChannel.get("beam");

}
public void loop(){

    telemetry.addData("beam", beam.getState());
    telemetry.update();
}
public void stop(){

}


}
