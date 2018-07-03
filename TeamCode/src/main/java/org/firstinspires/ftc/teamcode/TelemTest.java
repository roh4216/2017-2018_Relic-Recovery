package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by riseo on 10/28/2017.
 */

@Autonomous(name="TelemTest", group="OpMode")
//@Disabled

public class TelemTest extends LinearOpMode {

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

    Servo flick;

    Servo relicflip;
    Servo relicgrip;

    Servo jewel;

    CRServo depleft;
    CRServo depright;

    Servo grip;

    float liftpower;
    float leftdrive;
    float rightdrive;

    ColorSensor color;

   // OpticalDistanceSensor ods;
    OpticalDistanceSensor ods2;
    OpticalDistanceSensor ods3;

    DigitalChannel touch;
    DigitalChannel touch2;

    //ModernRoboticsI2cRangeSensor range;

    BNO055IMU gyro;

    Orientation angles;

    Acceleration accel;

    double heading;
    double pivotangle;

    double redtoblue;
    double bluetored;

    double red;
    double blue;

    Acceleration overallAccel;

    double accelX;
    double accelY;
    double accelZ;

    double distance = 0;

    boolean threesixty;

    int encoderTarget = 1500;

    double angleTarget;

    int case_switch = 300;

    int targetColumn = 3;

    int touchcount = 0;
    int touchcount2 = 0;

    boolean pressed = false;
    boolean pressed2 = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

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
       // range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        touch = hardwareMap.digitalChannel.get("touch");
        touch2 = hardwareMap.digitalChannel.get("touch2");


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

            int currentpos = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            accel = gyro.getLinearAcceleration();

            overallAccel = gyro.getOverallAcceleration();

            heading = angles.firstAngle;

            //accelX = accel.xAccel;
            accelY = overallAccel.yAccel;
           // accelZ = accel.zAccel;

            if(accelY > 0.08){
                distance += ((Math.pow(runtime.seconds(), 2))) * accelY;
            }

            else if(accelY < - 0.08){
                distance -= ((Math.pow(runtime.seconds(), 2))) * accelY;
            }

            else{
                runtime.reset();
            }

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);


                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    targetColumn = 1;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    targetColumn = 2;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    targetColumn = 3;
                }

            } else {
                telemetry.addData("VuMark", "not visible");
                targetColumn = 0;
            }

            red = (double) color.red();
            blue = (double) color.blue();


            if (blue < 1) {
                redtoblue = 100;
            } else if (blue > 1) {
                redtoblue = red / blue;
            }

            if (red < 1) {
                bluetored = 100;
            } else if (red > 1) {
                bluetored = blue / red;
            }


            if(touch.getState() && !pressed){
                touchcount += 1;
                pressed = true;
            }
            else if(!touch.getState()){
                pressed = false;
            }

            if(touch2.getState() && !pressed2){
                touchcount2 += 1;
                pressed2 = true;
            }
            else if(!touch2.getState()){
                pressed2 = false;
            }



            telemetry.addData("Touch State", touch.getState());
            telemetry.addData("Touch Count", touchcount);
            telemetry.addData("Pressed? ", pressed);

            telemetry.addData("Touch State2", touch2.getState());
            telemetry.addData("Touch Count2", touchcount2);
            telemetry.addData("Pressed2? ", pressed2);

           // telemetry.addData("raw ultrasonic", range.rawUltrasonic());
           // telemetry.addData("raw optical", range.rawOptical());
           // telemetry.addData("cm optical", "%.2f cm", range.cmOptical());
           // telemetry.addData("cm", "%.2f cm", range.getDistance(DistanceUnit.CM));



            telemetry.addData("Light: ", color.alpha());

            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("red : blue", redtoblue);
            telemetry.addData("blue : red", bluetored);

            telemetry.addData("Target Column", targetColumn);
            telemetry.addData("Case", case_switch);


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

}
