package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="blinkinTest", group="OpMode")
//@Disabled

public class blinkinTest extends OpMode {

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

        blinkin.setPosition(0.61);

    }


    @Override
    public void stop() {

    }

}

