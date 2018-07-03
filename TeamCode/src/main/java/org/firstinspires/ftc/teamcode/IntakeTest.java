package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="IntakeTest", group="OpMode")
//@Disabled

public class IntakeTest extends OpMode {

    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor driveLeft;
    DcMotor driveRight;

    private ElapsedTime runtime = new ElapsedTime();

    float leftdrive;
    float rightdrive;

    public IntakeTest() {
    }

    @Override
    public void init() {

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        driveLeft = hardwareMap.dcMotor.get("mc");
        driveRight = hardwareMap.dcMotor.get("md");

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        leftdrive = gamepad1.left_stick_y;
        leftdrive = Range.clip(leftdrive, -1, 1);
        rightdrive = gamepad1.right_stick_y;
        rightdrive = Range.clip(rightdrive, -1, 1);
        driveLeft.setPower(leftdrive);
        driveRight.setPower(rightdrive);


        if(gamepad1.b){
            intakeL.setPower(-0.5);
            intakeR.setPower(-0.5);
        }

        if(gamepad1.left_stick_button){
            intakeL.setPower(1);
            intakeR.setPower(1);
        }

        if(gamepad1.right_stick_button){
            intakeL.setPower(0);
            intakeR.setPower(0);
        }


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

