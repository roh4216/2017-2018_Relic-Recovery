package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@TeleOp(name="V2Drive", group="OpMode")
//@Disabled

public class V2Drive extends OpMode {

    //limit switch thing
    DigitalChannel touch;

    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor lift;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    //Servo flipL;
   // Servo flipR;
    Servo flip;

    Servo flick;

    Servo depleft;

    DcMotor extendo;

    Servo grip;

    Servo jewel;

    Servo relicflip;
    Servo relicgrip;

    Servo stop;

    //telemetry for limit switch
    int touchcount = 0;
    boolean pressed = false;

    boolean ypressed = false;
    boolean apressed = false;

    float liftpower;
    float leftdrive;
    float rightdrive;

    float extendofloat;

    float drivefactor = 1;

    double manualgrip;

    int lifttarget;

    double grippos = 0.8;

    double posL = 0.2;
    double posR = 0.8;

    boolean reset = false;
    boolean flipped = false;

    boolean stoppable = true;

    double intakePower = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public V2Drive() {
    }

    @Override
    public void init() {


        //limit switch thing
        touch = hardwareMap.digitalChannel.get("touch");

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        lift = hardwareMap.dcMotor.get("mc");
        extendo = hardwareMap.dcMotor.get("md");

        fl = hardwareMap.dcMotor.get("me");
        fr = hardwareMap.dcMotor.get("mf");
        bl = hardwareMap.dcMotor.get("mg");
        br = hardwareMap.dcMotor.get("mh");

        //flipL = hardwareMap.servo.get("sa");
       // flipR = hardwareMap.servo.get("sb");
        flip = hardwareMap.servo.get("sa");

        flick = hardwareMap.servo.get("sc");

        depleft = hardwareMap.servo.get("depleft");

        grip = hardwareMap.servo.get("sd");
        jewel = hardwareMap.servo.get("se");

        relicflip = hardwareMap.servo.get("sf");
        relicgrip = hardwareMap.servo.get("sg");

        stop = hardwareMap.servo.get("stop");

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        jewel.setPosition(1);
        flick.setPosition(0.1);
        flip.setPosition(0.25);
        //flipL.setPosition(0.2);
        //flipR.setPosition(0.8);
        grip.setPosition(0.7);
       // relicgrip.setPosition(0.4); // rev is .4
       // relicflip.setPosition(0.7);
        stop.setPosition(0);
        depleft.setPosition(.535);


        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void start(){

        relicflip.setPosition(0.85);

    }

    @Override
    public void loop() {
        intakeL.setPower(intakePower);
        intakeR.setPower(intakePower);

        //limit switch count stuff
        if(touch.getState() && !pressed){
            touchcount += 1;
            pressed = true;
        }
        else if(!touch.getState()){
            pressed = false;
        }

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

        liftpower = gamepad2.right_stick_y;
        liftpower = Range.clip(liftpower, -1, 1);
        lift.setPower(liftpower);

        extendofloat = gamepad2.left_stick_y;
        extendofloat = Range.clip(extendofloat, -1, 1);
        extendo.setPower(extendofloat);

        intakePower = Range.clip(intakePower, -1,1);

        if(gamepad1.left_trigger >= 0.01) {
            depleft.setPosition(0.175);
        }

        if(gamepad1.right_trigger >= 0.01) {
            depleft.setPosition(0.535);
        }
        if(gamepad1.left_trigger < 0.01 && gamepad1.right_trigger <0.01){
        //    depleft.setPower(0);
          //  depright.setPower(0);
        }

        if(gamepad1.b){
            intakePower  = -1;
        }

        if(gamepad1.left_stick_button){
           // intakePower = 0.8;
            intakePower = .6;
        } //was .9, changed to avoid jams

        if(gamepad1.right_stick_button){
            intakePower = 0;
        }


        if(gamepad1.a && !apressed){
            intakePower -= .1;
            apressed = true;
        }
        else if(!gamepad1.a) {
            apressed = false;
        }
        if(gamepad1.y && !ypressed){
            intakePower += 0.1;
            ypressed = true;
        }
        else if(!gamepad1.y){
            ypressed = false;
        }


        if(intakePower >= .9){
            stop.setPosition(0);
            stoppable = false;
        }
        else{
            stoppable = true;
        }

        if(gamepad2.dpad_up){
          //  flipL.setPosition(0.81);
          //  flipR.setPosition(0.19);
            flip.setPosition(0.865);

            //posL = 0.81;
            //posR = 0.19;
            flipped = true;
        }

        if(gamepad2.dpad_down){
           // flipL.setPosition(0.2);
           // flipR.setPosition(0.8);
            flip.setPosition(0.25);

            //posL = 0.13;
            //posR = 0.8;
            flipped = false;
        }

        //flipL.setPosition(posL);
        //flipR.setPosition(posR);


        if(gamepad2.right_bumper){
            grippos = .18; //for rev .22
        }
        if(gamepad2.left_bumper){
            grippos = 0.75; // for rev .7
        }

        if(gamepad2.y){
            //relicflip.setPosition(0);
            relicflip.setPosition(0.4);

        }

        if(gamepad2.a){
            relicflip.setPosition(0.86);
        }

        if(gamepad2.b){
           // relicgrip.setPosition(0);
            relicgrip.setPosition(0.4);

        }

        if(gamepad2.x){
            relicgrip.setPosition(0.805);
        }

        grip.setPosition(grippos);

        if(intakeL.getPower() >= .5 && intakeR.getPower() >= 0.5 && stoppable == true){
            stop.setPosition(.35);
        }
        if(intakeL.getPower() < .5 && intakeR.getPower() < 0.5 && stoppable == true){
            stop.setPosition(0);
        }
        /*if(gamepad1.dpad_down){

            intakeL.setPower(1);
            intakeR.setPower(1);

            stop.setPosition(0);
            stoppable = false;

        }
        if(gamepad1.dpad_up){
            stoppable = true;

            intakeL.setPower(0);
            intakeR.setPower(0);

        }*/


        //limit switch telemetry
        telemetry.addData("01 Intake Power",intakePower);
        telemetry.addData("Touch State", touch.getState());
        telemetry.addData("Touch Count", touchcount);
        telemetry.addData("Pressed? ", pressed);


        //telemetry.addData("LeftServo: ", flipL.getPosition());
        //telemetry.addData("RightServo: ", flipR.getPosition());
        telemetry.addData("FlipServo: ", flip.getPosition());

        telemetry.addData("posL: ", posL);
        telemetry.addData("posR: ", posR);

        telemetry.addData("Lift Pos", lift.getCurrentPosition());
        telemetry.addData("Flip Reset: ", reset);
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

