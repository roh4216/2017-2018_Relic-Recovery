package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="blinkinTest", group="OpMode")
//@Disabled

public class blinkinTestFullRange extends OpMode {

    Servo blinkin;

    double color;

    @Override
    public void init() {

       blinkin = hardwareMap.servo.get("blinkin");

    }

    @Override
    public void start(){



    }

    @Override
    public void loop() {

        blinkin.setPosition(0.7075);

    }


    @Override
    public void stop() {

    }

}

