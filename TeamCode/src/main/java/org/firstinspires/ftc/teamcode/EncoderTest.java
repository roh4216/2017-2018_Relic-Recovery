package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by riseo on 10/28/2017.
 */

@Autonomous(name="EncoderTest", group="OpMode")
//@Disabled

public class EncoderTest extends LinearOpMode {

    VuforiaLocalizer vuforia;

    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor lift;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    Servo flipL;
    Servo flipR;

    Servo rotate;

    Servo jewel;

    CRServo depleft;
    CRServo depright;

    Servo grip;

    float liftpower;
    float leftdrive;
    float rightdrive;

    ColorSensor color;

    BNO055IMU gyro;

    Orientation angles;

    double heading;
    double pivotangle;

    double redtoblue;
    double bluetored;

    double red;
    double blue;

    boolean threesixty;

    int encoderTarget = 1500;

    double angleTarget;

    int case_switch = 0;

    int targetColumn = 3;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        color = hardwareMap.colorSensor.get("cl");

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        lift = hardwareMap.dcMotor.get("mc");

        fl = hardwareMap.dcMotor.get("me");
        fr = hardwareMap.dcMotor.get("mf");
        bl = hardwareMap.dcMotor.get("mg");
        br = hardwareMap.dcMotor.get("mh");

        flipL = hardwareMap.servo.get("sa");
        flipR = hardwareMap.servo.get("sb");
        rotate = hardwareMap.servo.get("sc");
        grip = hardwareMap.servo.get("sd");
        jewel = hardwareMap.servo.get("se");

        depleft = hardwareMap.crservo.get("depleft");
        depright = hardwareMap.crservo.get("depright");

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate.setPosition(0);
        flipL.setPosition(0.28);
        flipR.setPosition(0.72);
        grip.setPosition(0.1);
        jewel.setPosition(0);

        BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();
        gyroparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        gyro.initialize(gyroparameters);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            int currentpos = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

            angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            heading = angles.firstAngle;

            engageDriveEncoders();

            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            switch(case_switch){

                case 0:

                    fl.setPower(0.1);
                    sleep(1000);
                    fl.setPower(0);
                    fr.setPower(0.1);
                    sleep(1000);
                    fr.setPower(0);
                    bl.setPower(0.1);
                    sleep(1000);
                    bl.setPower(0);
                    br.setPower(0.1);
                    sleep(1000);
                    br.setPower(0);

                    break;
            }

            telemetry.addData("Avg Encoder", currentpos);

            telemetry.addData("Front Left Encoder", fl.getCurrentPosition());
            telemetry.addData("Front Right Encoder", fr.getCurrentPosition());
            telemetry.addData("Back Left Encoder", bl.getCurrentPosition());
            telemetry.addData("Back Right Encoder", br.getCurrentPosition());

            telemetry.addData("Heading", heading);

            telemetry.addData("Encoder Target", encoderTarget);

            telemetry.update();

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
        //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        int currentpos = (/*fl.getCurrentPosition() + */fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
        double power = ((minspeed - maxspeed) / -target) * currentpos + maxspeed;

        //drive forward to target
        if(-currentpos < target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }

        //if overshoot, drive backwards
        else if(-currentpos > target){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }


        return power;
    }

    public double gyroDrive(double targetheading, double drivepower){
        double headingError = targetheading - heading;

        double baseTurnPowerFactor = 0.003;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.;

        double driveStraight;

        if (headingError > 0) {
            driveStraight = (headingError * baseTurnPowerFactor);
        }
        else {
            driveStraight = (headingError * baseTurnPowerFactor);
        }

        driveStraight = Range.clip(driveStraight, -0.2, 0.2);

        fl.setPower(driveStraight - drivepower);
        fr.setPower(-driveStraight - drivepower);
        bl.setPower(driveStraight - drivepower);
        br.setPower(-driveStraight - drivepower);

        return driveStraight;
    }

    public double gyroTurn(double targetheading, double error){
        double headingError = targetheading - heading;

        double baseTurnPowerFactor = 0.004;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.01;

        double driveSteering;

        if (headingError > 0) {
            driveSteering = (headingError * baseTurnPowerFactor) + baseTurnPowerMin;
        }
        else {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }

        // Clips the range of the steering and going straight
        driveSteering = Range.clip(driveSteering, -0.9, 0.9); //was 0.3 and -0.3

        if(heading > targetheading + error){
            fl.setPower(driveSteering);
            fr.setPower(-driveSteering);
            bl.setPower(driveSteering);
            br.setPower(-driveSteering);
        }
        else if(heading < targetheading - error){
            fl.setPower(driveSteering);
            fr.setPower(-driveSteering);
            bl.setPower(driveSteering);
            br.setPower(-driveSteering);
        }
        else{
            Drive(0);
        }

        return driveSteering;

    }

    public double gyro360(double targetheading, double error){
        double headingError = targetheading - heading;

        double baseTurnPowerFactor = 0.007;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.01;

        double driveSteering;

        if (headingError > 0) {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }
        else {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }

        // Clips the range of the steering and going straight
        driveSteering = Range.clip(driveSteering, -0.35, 0.35); //was 0.3 and -0.3

        if(heading < targetheading - error){
            fl.setPower(driveSteering);
            fr.setPower(-driveSteering);
            bl.setPower(driveSteering);
            br.setPower(-driveSteering);
        }
        else if(heading > targetheading + error){
            fl.setPower(-driveSteering);
            fr.setPower(driveSteering);
            bl.setPower(-driveSteering);
            br.setPower(driveSteering);
        }
        else{
            Drive(0);
        }

        return driveSteering;

    }


    public double gyro360right(double targetheading, double error){
        double headingError = targetheading - heading;

        double baseTurnPowerFactor = 0.007;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.01;

        double driveSteering;

        if (headingError > 0) {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }
        else {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }

        // Clips the range of the steering and going straight
        driveSteering = Range.clip(driveSteering, -0.35, 0.35); //was 0.3 and -0.3

        if(heading < targetheading - error){
            fl.setPower(-driveSteering);
            fr.setPower(driveSteering);
            bl.setPower(-driveSteering);
            br.setPower(driveSteering);
        }
        else if(heading > targetheading + error){
            fl.setPower(driveSteering);
            fr.setPower(-driveSteering);
            bl.setPower(driveSteering);
            br.setPower(-driveSteering);
        }
        else{
            Drive(0);
        }

        return driveSteering;

    }


}

