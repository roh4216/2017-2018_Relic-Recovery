package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Created by riseo on 3/29/2018.
 */
@TeleOp(name="TestBoi", group="OpMode")
@Disabled



public class TestBoi extends OpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    double leftpower;
    double rightpower;
    Servo flipL;
    Servo flipR;
    double servopos = 0;

    public TestBoi() {

    }

    @Override
    public void init(){
        fl= hardwareMap.dcMotor.get("ma");
        fr = hardwareMap.dcMotor.get("mb");
        bl = hardwareMap.dcMotor.get("mc");
        br = hardwareMap.dcMotor.get("md");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        flipL = hardwareMap.servo.get("ma");
        flipR = hardwareMap.servo.get("mb");

        flipL.setPosition(servopos);
        flipR.setPosition(servopos);
    }
    @Override
    public void loop(){

        flipL.setPosition(servopos);
        flipR.setPosition(servopos);

        leftpower = gamepad1.left_stick_y;
        rightpower = gamepad1.right_stick_y;

        fl.setPower(leftpower);
        fr.setPower(leftpower);
        bl.setPower(rightpower);
        br.setPower(rightpower);

        leftpower = Range.clip(leftpower,-1,1);
        rightpower = Range.clip(rightpower, -1, 1);

        if(gamepad1.y) {
            servopos = 1;
        }
        if(gamepad1.b){
            servopos = 0;
        }

    }
    @Override
    public void stop(){

    }
}
