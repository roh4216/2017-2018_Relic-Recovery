package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

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
 * Created by riseo on 3/24/2018.
 */

@Autonomous(name="MotionProfileTest", group="OpMode")
@Disabled

public class MotionProfileTest extends LinearOpMode {

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

    Servo jewel;
    Servo flick;

    CRServo depleft;
    CRServo depright;

    Servo relicflip;
    Servo relicgrip;

    Servo grip;

    float liftpower;
    float leftdrive;
    float rightdrive;

    ColorSensor color;

    //OpticalDistanceSensor ods;
    OpticalDistanceSensor ods2;
    OpticalDistanceSensor ods3;

    DigitalChannel touch;

    BNO055IMU gyro;

    Orientation angles;

    double heading;
    double pivotangle;

    double redtoblue;
    double bluetored;

    double red;
    double blue;

    boolean threesixty;

    boolean right = false;
    boolean left = false;

    boolean pressed = false;

    double startpos;

    int encoderTarget = 1500;

    int columnheading = 20;

    int pileheading;

    int recollectcase;

    int case_switch = 0;

    int targetColumn = 2;

    int touchcount = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intaketimer = new ElapsedTime();

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASwVfEf/////AAAAGVUcSPfGZkhat7cnpNByrMFdPw2Zr5xYc5xaR+/21AT1qWh/W+XJXAXeLawgPEoVaQgqyYN9GtPrwsFz5ohHqqV1ALLAf45k4c8Hl0rCwv3fuBRM68Tt1EAfjKfO0jxk/plPW3XkgHd9LGzH7OPmvKJ515OzLGYodHJgb23xEa4AB+JpoXRHrpslIYDEyARZaPQdNBOfi+jEjC10rD4DaC14gZ2ySxdpheAYmz3XlYTcA72SsfB/rMO11TJI+VWIR8D5rW7P2wIofjjRZ1XNIrWPYTe/HTZy8YanS53V23e1IEm/gsN7VCzBG0ob0NhrxO2NmevMjeYkqAPWar0dITY88sPdn7ZVfv9+iwcePx0O\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        color = hardwareMap.colorSensor.get("cl");
        //ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");
        ods3 = hardwareMap.opticalDistanceSensor.get("ods3");

        touch = hardwareMap.digitalChannel.get("touch");

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        lift = hardwareMap.dcMotor.get("mc");

        fl = hardwareMap.dcMotor.get("me");
        fr = hardwareMap.dcMotor.get("mf");
        bl = hardwareMap.dcMotor.get("mg");
        br = hardwareMap.dcMotor.get("mh");

        flipL = hardwareMap.servo.get("sa");
        flipR = hardwareMap.servo.get("sb");
        flick = hardwareMap.servo.get("sc");
        grip = hardwareMap.servo.get("sd");
        jewel = hardwareMap.servo.get("se");
        relicflip = hardwareMap.servo.get("sf");
        relicgrip = hardwareMap.servo.get("sg");

        depleft = hardwareMap.crservo.get("depleft");
        depright = hardwareMap.crservo.get("depright");

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();
        gyroparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        gyro.initialize(gyroparameters);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            if(touch.getState() && !pressed){
                touchcount += 1;
                pressed = true;
            }
            else if(!touch.getState()){
                pressed = false;
            }

            int currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

            angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            heading = angles.firstAngle;

            engageDriveEncoders();

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);


                if(vuMark == RelicRecoveryVuMark.LEFT){
                    targetColumn = 1;
                }

                else if(vuMark == RelicRecoveryVuMark.CENTER){
                    targetColumn = 2;
                }

                else if(vuMark == RelicRecoveryVuMark.RIGHT){
                    targetColumn = 3;
                }

            }
            else {
                telemetry.addData("VuMark", "not visible");
                targetColumn = 0;
            }

            red = (double) color.red();
            blue = (double) color.blue();


            if(blue < 1){
                redtoblue = 100;
            }
            else if(blue > 1){
                redtoblue = red / blue;
            }

            if(red < 1){
                bluetored = 100;
            }

            else if(red > 1){
                bluetored = blue / red;
            }



//****************CASES BEGIN HERE CASES BEGIN HERE CASES BEGIN HERE*****************************

            /*RED POS 1*/

            switch(case_switch){

                case 0:

                    resetDriveEncoders();
                    case_switch = 1;

                    break;

                case 1:

                    ScurveLong(3200, 0.8, 15);

                    if(currentpos > 3200 - 15){
                        case_switch = 2;
                    }

                    break;

                case 2:

                    Drive(0);
                    startpos = currentpos;
                    sleep(1500);
                    case_switch = 3;

                    break;

                case 3:

                    ScurveMed(1000, -0.4, 15, startpos);

                    if(currentpos < 1000 + 15){
                        case_switch = 8;
                    }


                    break;

                case 4:

                    Drive(0);
                    resetDriveEncoders();
                    sleep(1500);
                    case_switch = 5;

                    break;

                case 5:

                    //Scurve(-1000, -0.4);

                    if(currentpos < -1000){
                        case_switch = 6;
                    }

                    break;

                case 6:

                    Drive(0);
                    resetDriveEncoders();
                    sleep(1500);
                    case_switch = 7;

                    break;

                case 7:

                    //Scurve(-500, -0.4);

                    if(currentpos < -500){
                        case_switch = 8;
                    }


                    break;

                case 8:

                    Drive(0);

                    break;

            }


            // telemetry.addData("ODS", ods.getRawLightDetected());

            telemetry.addData("Case", case_switch);

            telemetry.addData("Avg Encoder", currentpos);

            telemetry.addData("Current speed", fl.getPower());

            telemetry.addData("Front Left Encoder", fl.getCurrentPosition());
            telemetry.addData("Front Right Encoder", fr.getCurrentPosition());
            telemetry.addData("Back Left Encoder", bl.getCurrentPosition());
            telemetry.addData("Back Right Encoder", br.getCurrentPosition());

            //telemetry.addData("Heading", heading);

            //telemetry.addData("Encoder Target", encoderTarget);

            telemetry.update();

        }

    }

    public double ScurveLong(double target, double targetV, double error){

        double currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;
        double k = 70 / target;

        double up = (-k * (currentpos - (target / 12)));
        double down = (k * (currentpos - (9 * target / 10)));
       // double backup = (-k * (currentpos - (target / 6)));
       // double backdown = (k * (currentpos - (4 * target / 5)));
        double minV = 0.035; //minimum needed to move


        if (currentpos < (target / 6)) {
            power = (targetV - (2 * minV)) / (1 + (Math.exp(up))) + (2 * minV);
        }

        else if (currentpos > (4 * target / 5)){
            power = (targetV - minV) / (1 +(Math.exp(down))) + minV;
        }

        else{
            power = targetV;
        }


        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);

        telemetry.addData("Power > ", power);

        return power;
    }

    public double ScurveMed(double target, double targetV, double error, double startpos){

        double currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;
        double t = startpos - target;
        double k = 70 / t;


        double up = (-k * ((currentpos - target) - (t / 6)));
        double down = (k * ((currentpos - target) - (5 * t / 6)));
        double backup = (-k * (currentpos - (target / 6)));
        double backdown = (k * (currentpos - (4 * target / 5)));
        double minV = 0.035; //minimum needed to move


        if(target < startpos){

            if (currentpos < (t / 3) + target) {
                power = (targetV + (2 * minV)) / (1 + (Math.exp(up))) - (2 * minV);
            }

            else if (currentpos > (2 * t / 3) + target){
                power = (targetV + minV) / (1 +(Math.exp(down))) - minV;
            }

            else{
                power = targetV;
            }

        }

        else{

            if (currentpos > (target / 3)) {
                power = (targetV + (5 * minV)) / (1 +(Math.exp(backup))) - (5 * minV);
            }

            else if (currentpos < (2 * target / 3)){
                power = (targetV + minV) / (1 +(Math.exp(backdown))) - minV;
            }

            else{
                power = targetV;
            }

        }



        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);

        telemetry.addData("Power > ", power);

        return power;
    }

    public double Parabolic(int target, double targetV){

        int currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;

        if(target > 0){
            if (currentpos < (target / 4)) {
                power = (-targetV / (Math.pow(0.25 * target, 2))) * Math.pow(currentpos - (0.25 * target), 2) + targetV + 0.05;
            }

            else if (currentpos > (3 * target / 4)){
                power = (-targetV / (Math.pow(0.25 * target, 2))) * Math.pow(currentpos - (0.75 * target), 2) + targetV + 0.01;
            }

            else{
                power = targetV;
            }
        }

        else{

            if (currentpos > (target / 4)) {
                power = (-targetV / (Math.pow(0.25 * target, 2))) * Math.pow(currentpos - (0.25 * target), 2) + targetV - 0.05;
            }

            else if (currentpos < (3 * target / 4)){
                power = (-targetV / (Math.pow(0.25 * target, 2))) * Math.pow(currentpos - (0.75 * target), 2) + targetV - 0.01;
            }

            else{
                power = targetV;
            }
        }


        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);


        return power;
    }

    public double TrapezoidMotionProfile(int target, double targetV){

        int currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;

        if(target > 0){
            if (currentpos < (target / 4)) {
                power = ((4 * targetV * currentpos) / target) + 0.05;
            }

            else if (currentpos > (3 * target / 4)){
                power = ((-4 * targetV / target) * (currentpos - target)) + 0.01;
            }

            else{
                power = targetV;
            }
        }

        else{
            if (currentpos > (target / 4)) {
                power = ((4 * targetV * currentpos) / target) - 0.05;
            }

            else if (currentpos < (3 * target / 4)){
                power = ((-4 * targetV / target) * (currentpos - target)) - 0.01;
            }

            else{
                power = targetV;
            }
        }


        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);

        //drive forward to target
     /*   if(currentpos < target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }*/

        //drive backwards to target
       /* else if(-currentpos > target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }*/


        return power;
    }

    public double TrapezoidMotionProfile2(int target, double targetV){

        int currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;

        if (currentpos < (target / 8)) {
            power = ((8 * targetV * currentpos) / target) + 0.05;
        }

        else if (currentpos > (7 * target / 8)){
            power = ((-8 * targetV / target) * (currentpos - target));
        }

        else{
            power = targetV;
        }

        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);

        //drive forward to target
     /*   if(currentpos < target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }*/

        //drive backwards to target
       /* else if(-currentpos > target){
            fl.setPower(-power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        }*/


        return power;
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


    public void resetDriveEncoders(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }






}
