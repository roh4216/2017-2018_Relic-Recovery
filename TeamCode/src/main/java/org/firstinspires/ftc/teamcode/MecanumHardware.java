package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="MecanumHardware", group="OpMode")
@Disabled

public class MecanumHardware extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor leftIntake;
    DcMotor rightIntake;

    DigitalChannel touch;

    int case_switch = 0;


    public MecanumHardware() {
        leftFront = hardwareMap.dcMotor.get("ma");
        rightFront = hardwareMap.dcMotor.get("mb");
        leftRear = hardwareMap.dcMotor.get("mc");
        rightRear = hardwareMap.dcMotor.get("md");
        leftIntake = hardwareMap.dcMotor.get("Li");
        rightIntake = hardwareMap.dcMotor.get("Ri");

        touch = hardwareMap.digitalChannel.get("touch");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection((DcMotor.Direction.REVERSE));
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

}
