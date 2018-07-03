package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SensorBotDrive", group="OpMode")
//@Disabled

public class SensorBotDrive extends OpMode {

    //limit switch thing

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    float leftdrive;
    float rightdrive;

    float drivefactor = 1;

    private ElapsedTime runtime = new ElapsedTime();

    public SensorBotDrive() {
    }

    @Override
    public void init() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        //fast mode
        if(gamepad1.left_bumper){
            drivefactor = 1;
        }

        //med mode
        if(gamepad1.right_bumper){
            drivefactor = 2;
        }

        //slow mode
        if(gamepad1.dpad_down){
            drivefactor = 3;
        }

        leftdrive = gamepad1.left_stick_y / drivefactor;
        leftdrive = Range.clip(leftdrive, -1, 1);

        fl.setPower(leftdrive);
        bl.setPower(leftdrive);

        rightdrive = gamepad1.right_stick_y / drivefactor;
        rightdrive = Range.clip(rightdrive, -1, 1);

        fr.setPower(rightdrive);
        br.setPower(rightdrive);


        telemetry.update();


    }


    @Override
    public void stop() {

    }

    //values to scale power
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}

