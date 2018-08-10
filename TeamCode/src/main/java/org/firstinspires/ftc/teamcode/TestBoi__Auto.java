package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by riseo on 4/7/2018.
 */
@Autonomous
@Disabled

public class TestBoi__Auto extends LinearOpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    double leftpower;
    double rightpower;
    Servo flipL;
    Servo flipR;
    double servopos = 0;


    int encodertarget;
    int case_switch;
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode(){

        fl = hardwareMap.dcMotor.get("ma");
        fr= hardwareMap.dcMotor.get("mb");
        bl = hardwareMap.dcMotor.get("mc");
        br = hardwareMap.dcMotor.get("md");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        flipL = hardwareMap.servo.get("ma");
        flipR = hardwareMap.servo.get("mb");

        flipL.setPosition(servopos);
        flipR.setPosition(servopos);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            int currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;



            switch (case_switch) {
                case 0:
                engageDriveEncoders();
                driveToTarget(1000, 1, 0);
                encodertarget = 1000;
                if(currentpos >= encodertarget){
                    resetDriveEncoders();
                    disengageDriveEncoders();
                    runtime.reset();
                 case_switch = 1;
                }

                break;

                case 1:
                    while(runtime.seconds() <= 2){
                        fl.setPower(-.5);
                        fr.setPower(.5);
                    }
                    if(runtime.seconds() > 2){
                        case_switch = 1;
                        resetDriveEncoders();
                    }
                    break;
                case 2:
                    driveToTarget(1000,1,0);
                    break;

            }
        }

    }
    private double Drive(double power){
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
        return -power;
    }

    public void engageDriveEncoders(){
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void disengageDriveEncoders(){
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetDriveEncoders(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public double setDriveTarget(int target){
        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);

        return target;
    }

    public double driveToTarget(int target, double maxspeed, double minspeed){

        int currentpos = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
        double power = ((minspeed - maxspeed) / -target) * currentpos + maxspeed;

        //drive forward to target
        if(-currentpos < target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }

        //drive backwards to target
        else if(-currentpos > target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }


        return power;
    }



}
