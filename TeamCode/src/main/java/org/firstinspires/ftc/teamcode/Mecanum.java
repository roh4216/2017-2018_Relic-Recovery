package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="MecanumDrive", group="OpMode")
//@Disabled

public class Mecanum extends OpMode {


    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor leftIntake;
    DcMotor rightIntake;

    public Mecanum() {
    }

    @Override
    public void init() {

        leftFront = hardwareMap.dcMotor.get("ma");
        rightFront =  hardwareMap.dcMotor.get("mb");
        leftRear = hardwareMap.dcMotor.get("mc");
        rightRear =  hardwareMap.dcMotor.get("md");
        leftIntake = hardwareMap.dcMotor.get("li");
        rightIntake = hardwareMap.dcMotor.get("ri");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection((DcMotor.Direction.REVERSE));
        rightIntake.setDirection((DcMotor.Direction.REVERSE));

    }



    @Override
    public void loop () {
        // Mecanum drive

        float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RFspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;

        LFspeed = Range.clip(LFspeed,-1, 1);
        LBspeed = Range.clip(LBspeed, -1, 1);
        RFspeed = Range.clip(RFspeed, -1, 1);
        RBspeed = Range.clip(RBspeed, -1, 1);

        leftFront.setPower(LFspeed);
        rightFront.setPower(RFspeed);
        leftRear.setPower(LBspeed/2);
        rightRear.setPower(RBspeed/2);


        if (gamepad1.right_trigger>0) {
            leftIntake.setPower(.8);
            rightIntake.setPower(.8);

        }else if (gamepad1.left_trigger>0) {
            leftIntake.setPower(-.8);
            rightIntake.setPower(-.8);

        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);

        }
    }

    @Override
    public void stop () {

    }

}


