package org.firstinspires.ftc.teamcode;
//hi
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Michael on 6/6/2017.
 */
@Autonomous(name = "Auto", group = "Sensor")
public class Auto extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    ElapsedTime runTime = new ElapsedTime();
    //our Pixy device
    @Override
    public void runOpMode() throws InterruptedException {
        //setting up Pixy to the hardware map

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


        runTime.reset();


        while(runTime.seconds()<2){
            fl.setPower(0.7);
            br.setPower(0.7);
            bl.setPower(0.7);
            fr.setPower(0.7);
        }
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        runTime.reset();
        while(runTime.seconds()<0.3){
            fl.setPower(-0.5);
            br.setPower(-0.5);
            bl.setPower(-0.5);
            fr.setPower(-0.5);
        }
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        }
    }
