package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
 * Created by riseo on 10/28/2017.
 */

@Autonomous(name="V2AutoOpenCV", group="OpMode")
//@Disabled

public class V2AutoOpenCV extends LinearOpMode {

    VuforiaLocalizer vuforia;

    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor lift;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

   // Servo flipL;
   // Servo flipR;
    Servo flip;

    Servo jewel;
    Servo flick;

    Servo stop;

    Servo depleft;

    Servo relicflip;
    Servo relicgrip;

    Servo grip;

    float liftpower;
    float leftdrive;
    float rightdrive;

    ColorSensor color;

    DigitalChannel beam;
    DigitalChannel touch;
   // DigitalChannel touch2;


    BNO055IMU gyro;

    Orientation angles;

    double heading;
    double pivotangle;

    double redtoblue;
    double bluetored;

    double red;
    double blue;

    double distance1;
    double distance2;

    double startpos;

    double error;

    boolean threesixty;

    boolean finished = false;

    boolean right = false;
    boolean left = false;

    boolean pressed = false;
   // boolean pressed2 = false;

    int encoderTarget = 1500;

    int columnheading = 20;

    int pileheading;

    int recollectcase;

    int case_switch = 300;

    int targetColumn = 2;

    int touchcount = 0;
  //  int touchcount2 = 0;



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime pressedtime = new ElapsedTime();
    //above timer used for JD's hopeful glyph jam fix talk to eric for info

//    JewelOpMode jewelDetector1 = new JewelOpMode();
    JewelDetector jewelDetector = new JewelDetector();


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

        beam = hardwareMap.digitalChannel.get("beam");
        touch = hardwareMap.digitalChannel.get("touch2");
        // touch2 = hardwareMap.digitalChannel.get("touch2");

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        lift = hardwareMap.dcMotor.get("mc");

        fl = hardwareMap.dcMotor.get("me");
        fr = hardwareMap.dcMotor.get("mf");
        bl = hardwareMap.dcMotor.get("mg");
        br = hardwareMap.dcMotor.get("mh");

        //flipL = hardwareMap.servo.get("sa");
        //flipR = hardwareMap.servo.get("sb");
        flick = hardwareMap.servo.get("sc");
        flip = hardwareMap.servo.get("sa");

        grip = hardwareMap.servo.get("sd");
        jewel = hardwareMap.servo.get("se");
        relicflip = hardwareMap.servo.get("sf");
        relicgrip = hardwareMap.servo.get("sg");
        stop = hardwareMap.servo.get("stop");

        depleft = hardwareMap.servo.get("depleft");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        flick.setPosition(0.9);
        //flipL.setPosition(0.2);
        //flipR.setPosition(0.8);
        flip.setPosition(0.25);
        depleft.setPosition(0.175);

        grip.setPosition(0.22);
        jewel.setPosition(.95);
        relicgrip.setPosition(0.81);
        relicflip.setPosition(0.85);

        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());


        BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();
        gyroparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        gyro.initialize(gyroparameters);

       int i = 0;

        while(i < 1){
            telemetry.addLine("A - RED Pos 1, B - RED Pos 2");
            telemetry.addLine("X - BLUE Pos 1, Y - BLUE Pos 2");

            if(gamepad1.a){
                case_switch = 0;
                telemetry.clearAll();
                telemetry.addLine("RED Pos 1");
                i = 1;
            }
            else if(gamepad1.b){
                case_switch = 200;
                telemetry.clearAll();
                telemetry.addLine("RED Pos 2");
                i = 1;
            }
            else if(gamepad1.x){
                case_switch = 100;
                telemetry.clearAll();
                telemetry.addLine("BLUE Pos 1");
                i = 1;
            }
            else if(gamepad1.y){
                case_switch = 300;
                telemetry.clearAll();
                telemetry.addLine("BLUE Pos 2");
                i = 1;
            }

            telemetry.update();
        }

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            if(touch.getState() && !pressed){
                touchcount += 1;
                pressed = true;
            }
            else if(!touch.getState()){
                pressed = false;
                pressedtime.reset();
            }
    /*        if(pressedtime.seconds() >= .2 && (case_switch == 108 || case_switch == 109)){
                intakeL.setPower(0);
                intakeR.setPower(0);
                sleep(10);
                intakeL.setPower(1);
                intakeR.setPower(1);
            } */
            //*hopefully* when glyph is jammed this while stop and restart the intake to unjam it



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

                relicTrackables.deactivate();
            jewelDetector.areaWeight = 0.02;
            jewelDetector.detectionMode = org.firstinspires.ftc.teamcode.JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
            //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
            jewelDetector.debugContours = true;
            jewelDetector.maxDiffrence = 15;
            jewelDetector.ratioWeight = 15;
            jewelDetector.minArea = 700;

            jewelDetector.enable();
            telemetry.addLine("OpenCV Initialized. YAY!!!   :)");
            telemetry.update();


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

                        CameraDevice.getInstance().setFlashTorchMode(true);

                        //extend jewel arm and deploy intake
                        flick.setPosition(0.45);

                        sleep(100);

                        jewel.setPosition(0.2);

                        depleft.setPosition(0.55);

                        case_switch = 1;

                        break;

                    case 1:

//                        if(runtime.seconds() > 0.8){
//
//                            //go to left
//                            if(bluetored > 1.2){
//                                flick.setPosition(0.75);
//                                case_switch = 2;
//                            }
//
//                            //go to right
//                            else if(redtoblue > 1.2){
//                                flick.setPosition(0.1);
//                                case_switch = 2;
//                            }
//
//                            if(runtime.seconds() > 2){
//                                flick.setPosition(0.42);
//
//                                if(bluetored > 1.2){
//                                    flick.setPosition(0.75);
//                                    case_switch = 2;
//                                }
//                                else if(redtoblue > 1.2){
//                                    flick.setPosition(0.1);
//                                    case_switch = 2;
//                                }
//                            }
//                        }


                        switch (jewelDetector.getCurrentOrder()){
                            case RED_BLUE:
                                flick.setPosition(0.1);
                                telemetry.addLine("Red Blue");
                                telemetry.update();
                                break;

                            case BLUE_RED:
                                flick.setPosition(0.75);
                                telemetry.addLine("Blue Red");
                                telemetry.update();
                                break;

                            case UNKNOWN:
                                telemetry.addLine("Unknown");
                                telemetry.update();
                                if(bluetored > 1.2){
                                flick.setPosition(0.75);
                                //case_switch = 2;
                                }
                                //go to right
                                else if(redtoblue > 1.2){
                                flick.setPosition(0.1);
                               // case_switch = 2;
                            }

                            break;
                            }


                        if(runtime.seconds() > 3){
                            jewel.setPosition(.95);
                            //case_switch = 2;
                        }


                        break;

                    case 2:

                        //pivot to target angle and read vuforia

                        jewel.setPosition(.95);
                        flick.setPosition(0.1);

                        //right
                        if(targetColumn == 3){
                            encoderTarget = 2000;
                            recollectcase = 30;
                            pileheading = -20;
                            right = true;
                        }
                        //center or unknown
                        else if(targetColumn == 2 || targetColumn == 0){
                            encoderTarget = 2000;
                            recollectcase = 10;
                            pileheading = 20;

                        }
                        //left
                        else if(targetColumn == 1){
                            encoderTarget = 2400;
                            recollectcase = 20;
                            pileheading = 20;

                        }

                        case_switch = 3;


                        break;

                    case 3:

                        CameraDevice.getInstance().setFlashTorchMode(false);

                        //turn to 90 degrees

                        gyroTurn(-90, 2);


                        if(heading > -92 && heading < -88){
                            Drive(0);
                            resetDriveEncoders();
                            case_switch = 4;
                        }

                        break;

                    case 4:

                        //drive specific encoder count to l, c, or r column depending on what was last seen by vuforia

                        Scurve(encoderTarget, 0.8, 20);

                        if(encoderTarget - 20 < currentpos){
                            Drive(0);
                            case_switch = 5;
                        }


                        break;

                    case 5:

                        //stop, turn towards cryptobox

                        if(right){
                            gyroTurn(-20, 2);
                            columnheading = -20;
                            if(heading < -18 && heading > -22){
                                Drive(0);
                                flip.setPosition(0.865);
                                resetDriveEncoders();
                                case_switch = 6;
                            }
                        }
                        else{
                            gyroTurn(20, 2);
                            columnheading = 20;
                            if(heading < 22 && heading > 18){
                                Drive(0);
                                flip.setPosition(0.865);
                                resetDriveEncoders();
                                case_switch = 6;
                            }
                        }

                        break;

                    case 6:

                        //lift glyph bed, back up specific encoder distance, release grips

                        flip.setPosition(0.865);

                        encoderTarget = -200;
                        Scurve(encoderTarget, -0.25, 20);

                        if(encoderTarget + 20 > currentpos){
                            Drive(0);
                            //open grips
                            grip.setPosition(0.7);
                            runtime.reset();
                            case_switch = 7;
                        }


                        break;

                    case 7:

                        //drive forward and back a few times to knock glyph into position

                        Drive(0.65);
                        sleep(250);
                        flip.setPosition(0.25);
                        Drive(-0.4);
                        sleep(500);

                        resetDriveEncoders();
                        intakeL.setPower(0.9);
                        intakeR.setPower(0.9);
                        runtime.reset();
                        resetDriveEncoders();
                        case_switch = 8;

                        break;

                    case 8:

                        //drive to pile until it has detected block

                        if(currentpos < 1100){
                            ScurveLong(2200, 0.9, 20);
                            stop.setPosition(0.35);

                        }
                        else if(currentpos > 1100){
                            stop.setPosition(0.35);
                            Drive(0.2);
                        }

                        if(touch.getState()){
                            Drive(-0.55);
                            sleep(350);
                            Drive(0);
                            runtime.reset();
                            case_switch = 9;
                        }


                        else if(currentpos > 3200){
                            Drive(0);
                            runtime.reset();
                            case_switch = 9;
                        }


                        break;


                case 9:

                    Drive(0.25);

                    if(touch.getState()){
                        Drive(-0.25);
                        finished = true;
                    }


                    if((touchcount > 1 && !pressed) || finished){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }


                    if(currentpos > 3200){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }


                    break;


                case 10:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 11;
                    }

                    break;

                case 11:

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(1250, -0.9, 20, startpos);
                    if(currentpos < 1250 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        stop.setPosition(0);
                        case_switch = 12;
                    }

                        break;

                case 12:

                    gyroDrive(-34, -0.2);

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -350){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 13;
                    }

                    break;

                case 20:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 21;
                    }


                case 21:

                    //left column going to right to deposit

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }
                   /* else if(runtime.seconds() > 3){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                    }*/

                    ScurveMed(1700, -0.7, 20, startpos);
                    if(currentpos < 1700 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        stop.setPosition(0);
                        case_switch = 22;
                    }



                    break;

                case 22:

                    gyroDrive(-32 , -0.2);

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -650){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 13;
                    }

                    break;

                case 30:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 31;
                    }

                    break;

                case 31:

                    //right column going to left to deposit

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }
                   /* else if(runtime.seconds() > 3){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                    }*/


                    ScurveMed(1800, -0.7, 20, startpos);
                    if(currentpos < 1800 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        stop.setPosition(0);
                        case_switch = 32;
                    }


                    break;

                case 32:

                    gyroDrive(34, -0.2);

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -1000){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 13;
                    }

                    break;


                    case 13:

                        grip.setPosition(0.22);
                        flip.setPosition(0.865);

                        encoderTarget = -250;

                        Scurve(encoderTarget, -0.25, 20);

                        if(encoderTarget + 20 > currentpos){
                            Drive(0);
                            grip.setPosition(0.5);
                            resetDriveEncoders();
                            runtime.reset();
                            case_switch = 14;
                        }



                        break;

                case 14:

                    fl.setPower(-0.15);
                    bl.setPower(-0.15);
                    fr.setPower(0.1);
                    br.setPower(0.1);
                    sleep(350);

                    Drive(0.4);
                    sleep(200);

                    flip.setPosition(0.25);
                    Drive(-0.45);
                    sleep(600);

                    Drive(0.4);
                    sleep(150);

                    case_switch = 15;

                    break;

                case 15:

                    Drive(0);
                    break;





              //BLUE POS 1 BLUE POS 1 BLUE POS 1 BLUE POS 1//



                case 100:

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    //extend jewel arm and deploy intake

                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    depleft.setPosition(0.535);

                    case_switch = 101;

                    break;

                case 101:

                    if(runtime.seconds() > 0.8){

                        if(bluetored > 1.2){
                            flick.setPosition(0.1);
                            case_switch = 102;
                        }
                        else if(redtoblue > 1.2){
                            flick.setPosition(0.9);
                            case_switch = 102;
                        }

                        if(runtime.seconds() > 2){
                            flick.setPosition(0.42);

                            if(bluetored > 1.2){
                                flick.setPosition(0.1);
                                case_switch = 102;
                            }
                            else if(redtoblue > 1.2){
                                flick.setPosition(0.9);
                                case_switch = 102;
                            }
                        }
                    }

                    if(runtime.seconds() > 3){
                        jewel.setPosition(.95);
                        case_switch = 102;
                    }


                    break;

                case 102:


                    //pivot to target angle and read vuforia


                    jewel.setPosition(.95);
                    flick.setPosition(0.1);


                    //right
                    if(targetColumn == 3){
                        encoderTarget = 2400;
                        recollectcase = 120;
                        pileheading = -20;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        encoderTarget = 2000;
                        recollectcase = 110;
                        pileheading = -20;
                    }
                    //left
                    else if(targetColumn == 1){
                        encoderTarget = 2000;
                        recollectcase = 130;
                        pileheading = 20;
                        left = true;
                    }


                    case_switch = 103;


                    break;

                case 103:

                    CameraDevice.getInstance().setFlashTorchMode(false);

                    //turn to 90 degrees + retract servo

                    gyroTurn(90, 2);

                    if(heading > 88 && heading < 92){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 104;
                    }

                    break;

                case 104:

                    //drive specific encoder count to l, c, or r column depending on what was last seen by vuforia

                    Scurve(encoderTarget, 0.8, 20);

                    if(encoderTarget - 20 < currentpos){
                        Drive(0);
                        case_switch = 105;
                    }

                    break;

                case 105:

                    //stop, turn towards cryptobox

                    if(left){
                        gyroTurn(20, 2);
                        columnheading = 20;
                        if(heading < 22 && heading > 18){
                            Drive(0);
                            flip.setPosition(0.865);
                            resetDriveEncoders();
                            case_switch = 106;
                        }
                    }

                    else{
                        gyroTurn(-20, 2);
                        columnheading = -20;
                        if(heading < -18 && heading > -22){
                            Drive(0);
                            flip.setPosition(0.865);
                            resetDriveEncoders();
                            case_switch = 106;
                        }
                    }


                    break;

                case 106:

                    //lift glyph bed, back up specific encoder distance, release grips

                    flip.setPosition(0.865);
                    encoderTarget = -200;
                    Scurve(encoderTarget, -0.25, 20);

                    if(encoderTarget + 20 > currentpos){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.7);
                        runtime.reset();
                        case_switch = 107;
                    }

                    break;

                case 107:

                    //drive forward and back a few times to knock glyph into position


                    Drive(0.65);
                    sleep(250);
                    flip.setPosition(0.25);
                    Drive(-0.4);
                    sleep(500);
                    intakeL.setPower(0.9);
                    intakeR.setPower(0.9);
                    runtime.reset();
                    resetDriveEncoders();
                    case_switch = 108;

                    break;

                case 108:

                    //drive to pile until it has detected block

                    if(currentpos < 1100){
                        ScurveLong(2200, 0.9, 20);
                        stop.setPosition(0.35);

                    }
                    else if(currentpos > 1100){
                        stop.setPosition(0.35);
                        Drive(0.2);
                    }

                    if(touch.getState()){
                        Drive(-0.55);
                        sleep(350);
                        Drive(0);
                        runtime.reset();
                        case_switch = 109;
                    }

                    else if(currentpos > 3200){
                        Drive(0);
                        runtime.reset();
                        case_switch = 109;
                    }

                    break;

                case 109:

                    Drive(0.25);

                    if(touch.getState()){
                        Drive(-0.25);
                        finished = true;
                    }

                    if((touchcount > 1 && !pressed) || finished){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }

                    if(currentpos > 3200){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }

                    break;

                case 110:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 111;
                    }

                    break;

                case 111:

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }
                  /*  else if(runtime.seconds() > 3){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                    }*/

                    ScurveMed(1300, -0.9, 20, startpos);
                    if(currentpos < 1300 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                        case_switch = 112;
                    }
                    break;

                case 112:

                    gyroDrive(33, -0.2);

                    intakeL.setPower(0);
                    intakeR.setPower(0);

                    if(currentpos < -350){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        runtime.reset();
                        case_switch = 113;
                    }

                    break;

                case 120:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 121;
                    }

                    else if(runtime.seconds() > 2.5){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 121;
                    }

                    break;

                case 121:

                    //right column going to left to deposit
                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }
                   /* else if(runtime.seconds() > 3){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                    }*/

                    ScurveMed(1800, -0.7, 20, startpos);
                    if(currentpos < 1800 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                        case_switch = 122;
                    }

                    break;

                case 122:

                    gyroDrive(33, -0.2);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }


                    if(currentpos < -650){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 113;
                    }


                    break;

                case 130:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        runtime.reset();
                        case_switch = 131;
                    }


                    break;

                case 131:

                    //left column going to right to deposit

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }
                   /* else if(runtime.seconds() > 3){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                    }*/


                    ScurveMed(1800, -0.7, 20, startpos);
                    if(currentpos < 1800 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                       // stop.setPosition(0);
                       // grip.setPosition(0.22);
                        case_switch = 132;
                    }

                    break;


                case 132:

                    gyroDrive(-34, -0.2);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }


                    if(currentpos < -1000){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        stop.setPosition(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 113;
                    }

                    break;

                case 113:

                    grip.setPosition(0.22);
                    flip.setPosition(0.865);

                    encoderTarget = -250;

                    Scurve(encoderTarget, -0.25, 20);

                    if(encoderTarget + 20 > currentpos){
                        Drive(0);
                        grip.setPosition(0.5);
                        resetDriveEncoders();
                        runtime.reset();
                        case_switch = 114;
                    }


                    break;

                case 114:

                    fl.setPower(0.1);
                    bl.setPower(0.1);
                    fr.setPower(-0.15);
                    br.setPower(-0.15);
                    sleep(350);

                    Drive(0.7);
                    sleep(200);

                    flip.setPosition(0.25);
                    Drive(-0.4);
                    sleep(900);

                    Drive(0.4);
                    sleep(150);


                    case_switch = 115;

                    break;

                case 115:

                    Drive(0);
                    break;







                    //RED POS 2 RED POS 2 RED POS 2 RED POS 2//


                case 200:

                    //extend jewel arm

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    case_switch = 201;

                    break;

                case 201:

                    //read color sensor, determine target angle left or right

                    if(runtime.seconds() > 0.8){

                        if(bluetored > 1.2){
                            flick.setPosition(0.75);
                            case_switch = 202;
                        }
                        else if(redtoblue > 1.2){
                            flick.setPosition(0.1);
                            case_switch = 202;
                        }

                        if(runtime.seconds() > 2){
                            flick.setPosition(0.42);

                            if(bluetored > 1.2){
                                flick.setPosition(0.75);
                                case_switch = 202;
                            }
                            else if(redtoblue > 1.2){
                                flick.setPosition(0.1);
                                case_switch = 202;
                            }
                        }
                    }

                    if(runtime.seconds() > 2){
                        jewel.setPosition(.95);
                        case_switch = 202;
                    }



                    break;

                case 202:

                    //pivot to target angle and read vuforia

                    jewel.setPosition(.95);
                    flick.setPosition(0.1);

                    //right
                    if(targetColumn == 3){
                        columnheading = 100;
                        pileheading = 60;
                        encoderTarget = -1000;
                        recollectcase = 230;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        columnheading = 108;
                        pileheading = 54;
                        encoderTarget = -1100;
                        recollectcase = 210;
                    }
                    //left
                    else if(targetColumn == 1){
                        columnheading = 116;
                        pileheading = 60;
                        encoderTarget = -1400;
                        recollectcase = 220;
                    }

                    case_switch = 203;

                    break;

                case 203:

                    CameraDevice.getInstance().setFlashTorchMode(false);

                    //turn to 90 degrees + retract servo
                    gyroTurn(columnheading, 2);

                    if(heading > (columnheading - 2) && heading < (columnheading + 2)){
                        Drive(0);
                        resetDriveEncoders();
                        intaketimer.reset();
                        runtime.reset();
                        case_switch = 204;
                    }

                    break;

                case 204:

                    //bring down intake, back up

                    depleft.setPosition(.55);

                    error = 15;

                    Scurve(encoderTarget, -0.7, error);

                    if(encoderTarget + error > currentpos){
                        Drive(0);
                        resetDriveEncoders();
                        flip.setPosition(0.865);

                        case_switch = 205;
                    }


                    break;

                case 205:

                    //place glyph

                    flip.setPosition(0.865);

                    encoderTarget = -450;
                    Scurve(encoderTarget, -0.28, 20);

                    if(encoderTarget + error > currentpos){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.7);
                        runtime.reset();
                        case_switch = 206;
                    }

                    break;

                case 206:

                    Drive(0.55);
                    sleep(350);

                    flip.setPosition(0.25);

                    Drive(-0.62);
                    sleep(800);
                    Drive(0.3);
                    sleep(200);

                    resetDriveEncoders();
                    stop.setPosition(0.35);
                    intakeL.setPower(1);
                    intakeR.setPower(1);
                    runtime.reset();
                    case_switch = 207;

                    break;

                case 207:

                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 208;
                    }

                    break;

                case 208:

                    //drive to pile until gets glyph or goes too far

                    if(currentpos < 1100){
                        ScurveLong(2200, 0.9, 20);
                        stop.setPosition(0.35);

                    }
                    else if(currentpos > 1100){
                        stop.setPosition(0.35);
                        Drive(0.2);
                    }

                    if(touch.getState()){
                        Drive(-0.55);
                        sleep(350);
                        Drive(0);
                        runtime.reset();
                        case_switch = 209;
                    }


                    else if(currentpos > 3500){
                        Drive(0);
                        runtime.reset();
                        case_switch = 209;
                    }

                    break;

                case 209:

                    Drive(0.25);

                    if(touch.getState()){
                        Drive(-0.25);
                        finished = true;
                    }


                    if((touchcount > 1 && !pressed) || finished){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }


                    if(currentpos > 3500){
                        Drive(0);
                        runtime.reset();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }

                    break;

                case 210:

                    gyroTurn(pileheading, 2);

                    if(runtime.seconds() < 0.05){
                    intakeL.setPower(-0.5);
                    intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 211;
                    }

                    break;

                case 211:

                    //center start

                    if(runtime.seconds() > 1.2) {
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }


                    ScurveMed(1900, -0.9, 20, startpos);
                    if(currentpos < 1900 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 212;
                    }

                    break;

                case 212:

                    gyroDrive(185, -0.6);

                    if(runtime.seconds() > 1.2){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -650){
                        // Drive(0);
                        if(100 > heading && heading > 93){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 213;
                        }
                        else{
                            gyroTurn(96, 2);
                            if(98 > heading && heading > 94) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 213;
                            }
                        }
                    }

                    break;


                case 220:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading - 20, 2);

                    if(heading > pileheading - 20 - 2 && heading < pileheading - 20 + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 221;
                    }


                    break;


                case 221:

                    //left start

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(1450, -0.75, 20, startpos);
                    if(currentpos < 1450 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 222;
                    }



                    break;



                case 222:

                    gyroDrive(75, -0.6);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -900){
                        // Drive(0);
                        if(85 > heading && heading > 65){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 213;
                        }
                        else{
                            gyroTurn(75, 2);
                            if(77 > heading && heading > 73) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 213;
                            }
                        }
                    }

                    break;

                case 230:


                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }


                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 231;
                    }

                case 231:

                    //right start

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(2300, -0.9, 20, startpos);
                    if(currentpos < 2300 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 232;
                    }

                    break;



                case 232:

                    gyroDrive(175, -0.6);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -650){
                        // Drive(0);
                        if(98 > heading && heading > 91){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 213;
                        }
                        else{
                            gyroTurn(94, 2);
                            if(94 > heading && heading > 92) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 213;
                            }
                        }
                    }

                    break;

                case 213:

                    stop.setPosition(0);
                    grip.setPosition(0.22);
                    flip.setPosition(0.865);

                    encoderTarget = -300;

                    Scurve(encoderTarget, -0.12, 15);

                    if(encoderTarget + 15 > currentpos){
                        Drive(0);
                        grip.setPosition(0.5);
                        resetDriveEncoders();
                        runtime.reset();
                        case_switch = 214;
                    }


                    break;


                case 214:

                    if(recollectcase == 220) {
                        fl.setPower(-0.15);
                        bl.setPower(-0.15);
                        fr.setPower(0.1);
                        br.setPower(0.1);
                    }

                    else{
                        fl.setPower(0.1);
                        bl.setPower(0.1);
                        fr.setPower(-0.15);
                        br.setPower(-0.15);
                    }


                    sleep(350);

                    Drive(0.35);
                    sleep(300);

                    flip.setPosition(0.25);

                    Drive(-0.3);
                    sleep(1800);

                    Drive(0.6);
                    sleep(250);


                    case_switch = 215;

                    break;

                case 215:

                    Drive(0);
                    break;






//BLUE POS 2 BLUE POS 2 BLUE POS 2









                case 300:

                    //extend jewel arm, light on

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    case_switch = 301;

                    break;

                case 301:

                    //read color sensor, determine target angle left or right

                    if(runtime.seconds() > 0.8){

                        if(bluetored > 1.2){
                            flick.setPosition(0.1);
                            case_switch = 302;
                        }
                        else if(redtoblue > 1.2){
                            flick.setPosition(0.9);
                            case_switch = 302;
                        }

                        if(runtime.seconds() > 2){
                            flick.setPosition(0.42);

                            if(bluetored > 1.2){
                                flick.setPosition(0.1);
                                case_switch = 302;
                            }
                            else if(redtoblue > 1.2){
                                flick.setPosition(0.9);
                                case_switch = 302;
                            }
                        }
                    }

                    if(runtime.seconds() > 2){
                        jewel.setPosition(.95);
                        case_switch = 302;
                    }



                    break;

                case 302:

                    //pivot to target angle and read vuforia

                    jewel.setPosition(.95);
                    flick.setPosition(0.1);

                    //right
                    if(targetColumn == 3){
                    columnheading = -116;
                    pileheading = -60;
                    encoderTarget = -1400;
                    recollectcase = 320;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        columnheading = -108;
                        pileheading = -54;
                        encoderTarget = -1100;
                        recollectcase = 310;
                    }
                    //left
                    else if(targetColumn == 1){
                        columnheading = -100;
                        pileheading = -60;
                        encoderTarget = -1000;
                        recollectcase = 330;
                    }

                    case_switch = 303;

                    break;

                case 303:

                    CameraDevice.getInstance().setFlashTorchMode(false);

                    //turn to 90 degrees + retract servo
                    gyroTurn(columnheading, 2);

                    if(heading > (columnheading - 2) && heading < (columnheading + 2)){
                        Drive(0);
                        resetDriveEncoders();
                        intaketimer.reset();
                        runtime.reset();
                        case_switch = 304;
                    }

                    break;

                case 304:

                    //bring down intake, back up

                    depleft.setPosition(.55);

                    error = 15;

                    Scurve(encoderTarget, -0.7, error);

                    if(encoderTarget + error > currentpos){
                        Drive(0);
                        resetDriveEncoders();
                        flip.setPosition(0.865);

                        case_switch = 305;
                    }

                  /*  else if(-currentpos < encoderTarget - 20){
                        Drive(0);
                        resetDriveEncoders();
                        flipL.setPosition(0.81);
                        flipR.setPosition(0.19);
                        case_switch = 305;
                    }*/

                    break;

                case 305:

                    //place glyph

                    flip.setPosition(0.865);

                    encoderTarget = -450;
                    Scurve(encoderTarget, -0.28, 20);

                    if(encoderTarget + error > currentpos){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.7);
                        runtime.reset();
                        case_switch = 306;
                    }

                   /* else if(-currentpos < encoderTarget - 20){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.7);
                        runtime.reset();
                        case_switch = 306;
                    }*/
                    break;

                case 306:

                    //forward and back

                    Drive(0.55);
                    sleep(350);

                    flip.setPosition(0.25);

                    Drive(-0.62);
                    sleep(800);
                    Drive(0.3);
                    sleep(200);

                    resetDriveEncoders();
                    stop.setPosition(0.35);
                    intakeL.setPower(1);
                    intakeR.setPower(1);
                    runtime.reset();
                    case_switch = 307;

                    break;

                case 307:

                    //turn to pile

                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 308;
                    }

                    break;

                case 308:

                    //drive to pile until gets glyph or goes too far

                    if(currentpos < 2200){
                        ScurveLong(2200, 0.8, 20);
                    }
                    else if(currentpos > 1100 && fl.getPower() <= 0.4){
                        Drive(0.4);
                    }

                    if(touch.getState()){
                        Drive(-0.35);
                        sleep(350);
                        Drive(0);
                        runtime.reset();
                        case_switch = 309;
                    }

                 /*   if(!beam.getState()){
                        Drive(-0.1);
                        if(beam.getState()){
                            Drive(0);
                            distance1 = currentpos;
                            encoderTarget = 3200 - (int) distance1;
                            resetDriveEncoders();
                            runtime.reset();
                            case_switch = 309;
                        }
                    }*/

                    else if(currentpos > 3200){
                        Drive(0);
                        runtime.reset();
                        case_switch = 309;
                    }

                    break;

                case 309:

                    Drive(0.25);

                    if(touch.getState()){
                        Drive(-0.25);
                        finished = true;
                    }

                    if((touchcount > 1 && !pressed) || finished && !pressed){
                        Drive(0);
                        runtime.reset();
                        stop.setPosition(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }

                    if(currentpos > 3200){
                        Drive(0);
                        runtime.reset();
                        stop.setPosition(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = recollectcase;
                    }

                   /* if(touchcount > 1 && !pressed){
                        Drive(0);
                        grip.setPosition(0.22);
                        runtime.reset();
                        case_switch = recollectcase;
                    }

                    if(runtime.seconds() > 0.5){
                        Drive(0);
                    }

                    if(runtime.seconds() > 0.9){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        case_switch = recollectcase;
                    }*/
                    break;

                case 310:

                    //center start

                    /*if(runtime.seconds() > 1 && !pressed){
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }
                    else if(runtime.seconds() > 0.3 && pressed){
                        intakeL.setPower(1);
                        intakeR.setPower(1);
                    }*/
                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 311;
                    }


                   /* gyroDrive(pileheading, -0.4);

                    if(currentpos < 1600){
                        Drive(0);
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        resetDriveEncoders();
                        case_switch = 311;
                    }*/

                    break;



                case 311:

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(2000, -0.9, 20, startpos);
                    if(currentpos < 2000 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 312;
                    }


                    break;

                case 312:

                    gyroDrive(-170, -0.6);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -650){
                       // Drive(0);
                        if(-91 > heading && heading > -98){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 313;
                        }
                       else{
                            gyroTurn(-94, 2);
                            if(-92 > heading && heading > -96) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 313;
                            }
                        }
                    }

                    break;



                case 320:

                    //right start

                    if(runtime.seconds() < 0.05){
                        intakeL.setPower(-0.5);
                        intakeR.setPower(-0.5);
                    }

                    else{
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                    }

                    gyroTurn(pileheading + 20, 2);

                    if(heading > pileheading + 20 - 2 && heading < pileheading + 20 + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 321;
                    }

                    break;



                case 321:

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(1450, -0.75, 20, startpos);
                    if(currentpos < 1450 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        case_switch = 322;
                    }



                    break;

                case 322:

                    gyroDrive(-75, -0.6);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -900){
                        // Drive(0);
                        if(-65 > heading && heading > -85){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 313;
                        }
                        else{
                            gyroTurn(-75, 2);
                            if(-73 > heading && heading > -77) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 313;
                            }
                        }
                    }

                    break;


                case 330:

                    //left start

                    gyroTurn(pileheading, 2);

                    if(heading > pileheading - 2 && heading < pileheading + 2){
                        Drive(0);
                        startpos = currentpos;
                        case_switch = 331;
                    }

                    break;



                case 331:

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    ScurveMed(2300, -0.9, 20, startpos);
                    if(currentpos < 2300 + 20){
                        Drive(0);
                        resetDriveEncoders();
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        grip.setPosition(0.22);
                        case_switch = 332;
                    }



                    break;


                case 332:

                    gyroDrive(-175, -0.6);

                    if(runtime.seconds() > 0.5){
                        intakeL.setPower(0.5);
                        intakeR.setPower(0.5);

                    }

                    if(currentpos < -650){
                        // Drive(0);
                        if(-91 > heading && heading > -98){
                            Drive(0);
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            grip.setPosition(0.22);
                            resetDriveEncoders();
                            case_switch = 313;
                        }
                        else{
                            gyroTurn(-94, 2);
                            if(-92 > heading && heading > -96) {
                                Drive(0);
                                intakeL.setPower(0);
                                intakeR.setPower(0);
                                grip.setPosition(0.22);
                                resetDriveEncoders();
                                case_switch = 313;
                            }
                        }
                    }

                    break;



                case 313:

                    stop.setPosition(0);

                    lift.setPower(0);
                    grip.setPosition(0.22);
                    flip.setPosition(0.865);


                    encoderTarget = -300;

                    Scurve(encoderTarget, -0.15, 15);

                    if(encoderTarget + 15 > currentpos){
                        Drive(0);
                        grip.setPosition(0.5);
                        resetDriveEncoders();
                        runtime.reset();
                        case_switch = 314;
                    }

                  /*  else if(-currentpos < encoderTarget - 25){
                        driveToTarget(encoderTarget, 0.15, 0.1);
                    }*/


                    break;

                case 314:

                    if(recollectcase == 320) {
                        fl.setPower(-0.15);
                        bl.setPower(-0.15);
                        fr.setPower(0.1);
                        br.setPower(0.1);
                    }

                    else{
                        fl.setPower(0.1);
                        bl.setPower(0.1);
                        fr.setPower(-0.15);
                        br.setPower(-0.15);
                    }



                    sleep(350);

                    Drive(0.35);
                    sleep(300);

                    flip.setPosition(0.25);

                    Drive(-0.3);
                    sleep(1800);

                    Drive(0.6);
                    sleep(250);

                    case_switch = 315;

                    break;

                case 315:

                    Drive(0);
                    break;



            }


           // telemetry.addData("ODS", ods.getRawLightDetected());
            telemetry.addData("Case", case_switch);
            telemetry.addData("Avg Encoder", currentpos);
            telemetry.addData("Encoder Target", encoderTarget);
            telemetry.addData("Touch State", touch.getState());
            telemetry.addData("Touch Count", touchcount);
           // telemetry.addData("Beam", beam.getState());
            telemetry.addData("Pressed? ", pressed);
            telemetry.addData("Heading", heading);

            telemetry.addData("Timer: ", runtime.seconds());
            telemetry.addData("Intake timer: ", intaketimer.seconds());

            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("red : blue", redtoblue);
            telemetry.addData("blue : red", bluetored);

            telemetry.addData("Target Column", targetColumn);

            telemetry.addData("Front Left Encoder", fl.getCurrentPosition());
            telemetry.addData("Front Right Encoder", fr.getCurrentPosition());
            telemetry.addData("Back Left Encoder", bl.getCurrentPosition());
            telemetry.addData("Back Right Encoder", br.getCurrentPosition());





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
        double down = ((50/t) * ((currentpos - target) - (7 * t / 8)));
        double backup = (-k * (currentpos - (target / 6)));
        double backdown = (k * (currentpos - (4 * target / 5)));
        double minV = 0.03; //minimum needed to move


        if(target < startpos){

            if (currentpos < (t / 3) + target) {
                power = (targetV + (minV)) / (1 + (Math.exp(up))) - (minV);
            }

            else if (currentpos > (3 * t / 4) + target){
                power = (targetV + (4 * minV)) / (1 +(Math.exp(down))) - (4 * minV);
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

    public double Scurve(double target, double targetV, double error){

        double currentpos = -(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

        double power;
        double k = 40 / target;

        double up = (-k * (currentpos - (target / 6)));
        double down = (k * (currentpos - (4 * target / 5)));
        double backup = (-k * (currentpos - (target / 6)));
        double backdown = (k * (currentpos - (4 * target / 5)));
        double minV = 0.035; //minimum needed to move


        if(target > 0){

            if (currentpos < (target / 3)) {
                power = (targetV - (5 * minV)) / (1 + (Math.exp(up))) + (5 * minV);
            }

            else if (currentpos > (2 * target / 3)){
                power = (targetV - minV) / (1 +(Math.exp(down))) + minV;
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
        double baseTurnPowerMin = 0.04;

        double driveSteering;

        if (headingError > 0) {
            driveSteering = (headingError * baseTurnPowerFactor) + baseTurnPowerMin;
        }
        else {
            driveSteering = (headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }

        // Clips the range of the steering and going straight
        driveSteering = Range.clip(driveSteering, -0.98, 0.98); //was 0.3 and -0.3

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
        driveSteering = Range.clip(driveSteering, -0.9, 0.9); //was 0.3 and -0.3

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

