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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
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

@Autonomous(name="V2Auto", group="OpMode")
//@Disabled

public class V2Auto extends LinearOpMode {


    VuforiaLocalizer vuforia;

    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor lift;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    //Servo flipL;
    //Servo flipR;
    Servo flip;

    Servo jewel;
    Servo flick;

    Servo depleft;

    Servo relicflip;
    Servo relicgrip;

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

        intakeL = hardwareMap.dcMotor.get("ma");
        intakeR =  hardwareMap.dcMotor.get("mb");
        lift = hardwareMap.dcMotor.get("mc");

        fl = hardwareMap.dcMotor.get("me");
        fr = hardwareMap.dcMotor.get("mf");
        bl = hardwareMap.dcMotor.get("mg");
        br = hardwareMap.dcMotor.get("mh");

        //flipL = hardwareMap.servo.get("sa");
        //flipR = hardwareMap.servo.get("sb");
        flip = hardwareMap.servo.get("sa");

        flick = hardwareMap.servo.get("sc");
        grip = hardwareMap.servo.get("sd");
        jewel = hardwareMap.servo.get("se");
        relicflip = hardwareMap.servo.get("sf");
        relicgrip = hardwareMap.servo.get("sg");

        depleft = hardwareMap.servo.get("depleft");

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        flick.setPosition(0.9);
        flip.setPosition(0.25);
        //flipL.setPosition(0.2);
        //flipR.setPosition(0.8);
        grip.setPosition(0.22);
        jewel.setPosition(1);
        relicgrip.setPosition(0.4);
        relicflip.setPosition(0.7);
        depleft.setPosition(0.175);

        BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();
        gyroparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        gyro.initialize(gyroparameters);

        int i = 0;

        if(gamepad1.start){
            i = 0;
            telemetry.clearAll();
        }

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

            int currentpos = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

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

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    //extend jewel arm and deploy intake
                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    depleft.setPosition(.55);

                    case_switch = 1;

                    break;

                case 1:

                    if(runtime.seconds() > 0.8){

                        if(bluetored > 1.2){
                            flick.setPosition(0.9);
                            case_switch = 2;
                        }
                        else if(redtoblue > 1.2){
                            flick.setPosition(0.1);
                            case_switch = 2;
                        }

                        if(runtime.seconds() > 2){
                            flick.setPosition(0.42);

                            if(bluetored > 1.2){
                                flick.setPosition(0.9);
                                case_switch = 2;
                            }
                            else if(redtoblue > 1.2){
                                flick.setPosition(0.1);
                                case_switch = 2;
                            }
                        }
                    }

                    if(runtime.seconds() > 3){
                        jewel.setPosition(1);
                        //case_switch = 2;
                    }

                    break;

                case 2:

                    //pivot to target angle and read vuforia

                    jewel.setPosition(1);
                    flick.setPosition(0.1);

                    //right
                    if(targetColumn == 3){
                        encoderTarget = 1500;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        encoderTarget = 2000;
                    }
                    //left
                    else if(targetColumn == 1){
                        encoderTarget = 2500;
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

                    driveToTarget(encoderTarget, 0.3, 0.07);

                    if((encoderTarget - 20 < -currentpos && -currentpos < (encoderTarget + 20))){
                        Drive(0);
                        case_switch = 5;
                    }

                    break;

                case 5:

                    //stop, turn towards cryptobox

                    gyroTurn(20, 2);

                    if(heading < 22 && heading > 18){
                        Drive(0);
                        flip.setPosition(0.865);
                        //flipL.setPosition(0.81);
                        //flipR.setPosition(0.19);
                        resetDriveEncoders();
                        case_switch = 6;
                    }

                    break;

                case 6:

                    //lift glyph bed, back up specific encoder distance, release grips
                    flip.setPosition(0.865);

                    //flipL.setPosition(0.81);
                    //flipR.setPosition(0.19);
                    encoderTarget = -150;
                    driveToTarget(encoderTarget, 0.35, 0.3);

                    Drive(-0.2);

                    if(encoderTarget - 100 < -currentpos && -currentpos < encoderTarget + 100){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.5);
                        runtime.reset();
                        case_switch = 7;
                    }


                    break;

                case 7:

                    //drive forward and back a few times to knock glyph into position
                    flip.setPosition(0.25);

                    //flipL.setPosition(0.2);
                    //flipR.setPosition(0.8);

                    Drive(0.15);
                    sleep(500);
                    Drive(-0.15);
                    sleep(1200);
                    Drive(0.15);
                    sleep(1000);

                    case_switch = 8;

                    break;

                case 8:

                    //stop
                    Drive(0);

                    break;


                /*BLUE POS 1**************************************************************/


                case 100:

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    //extend jewel arm and deploy intake

                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    depleft.setPosition(.55);

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
                        jewel.setPosition(1);
                        case_switch = 102;
                    }


                    break;

                case 102:

                    //pivot to target angle and read vuforia


                    jewel.setPosition(1);
                    flick.setPosition(0.1);

                    //left
                    if(targetColumn == 1){
                        encoderTarget = 1500;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        encoderTarget = 2000;
                    }
                    //right
                    else if(targetColumn == 3){
                        encoderTarget = 2500;
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

                    driveToTarget(encoderTarget, 0.3, 0.07);

                    if((encoderTarget - 20 < -currentpos && -currentpos < (encoderTarget + 20))){
                        Drive(0);
                        case_switch = 105;
                    }

                    break;

                case 105:

                    //stop, turn towards cryptobox

                    gyroTurn(-20, 2);

                    if(heading < -18 && heading > -22){
                        Drive(0);
                        flip.setPosition(0.865);
                        //flipL.setPosition(0.81);
                        //flipR.setPosition(0.19);
                        resetDriveEncoders();
                        case_switch = 106;
                    }

                    break;

                case 106:

                    //lift glyph bed, back up specific encoder distance, release grips

                    flip.setPosition(0.865);

                   //flipL.setPosition(0.81);
                   // flipR.setPosition(0.19);

                    encoderTarget = -400;
                    driveToTarget(encoderTarget, 0.3, 0.1);

                    Drive(-0.2);

                    if(encoderTarget - 100 < -currentpos && -currentpos < encoderTarget + 100){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.5);
                        runtime.reset();
                        case_switch = 107;
                    }


                    break;

                case 107:

                    //drive forward and back a few times to knock glyph into position

                    flip.setPosition(0.25);

                    //flipL.setPosition(0.2);
                    //flipR.setPosition(0.8);

                    Drive(0.15);
                    sleep(500);
                    Drive(-0.15);
                    sleep(1200);
                    Drive(0.15);
                    sleep(1000);

                    case_switch = 108;

                    break;

                case 108:

                    //stop
                    Drive(0);

                    break;


                /*RED POSITION 2*******************************************************************/

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
                            flick.setPosition(0.9);
                            case_switch = 202;
                        }
                        else if(redtoblue > 1.2){
                            flick.setPosition(0.1);
                            case_switch = 202;
                        }

                        if(runtime.seconds() > 2){
                            flick.setPosition(0.42);

                            if(bluetored > 1.2){
                                flick.setPosition(0.9);
                                case_switch = 202;
                            }
                            else if(redtoblue > 1.2){
                                flick.setPosition(0.1);
                                case_switch = 202;
                            }
                        }
                    }

                    if(runtime.seconds() > 2){
                        jewel.setPosition(1);
                        case_switch = 202;
                    }



                    break;

                case 202:

                    //pivot to target angle and read vuforia

                    jewel.setPosition(1);
                    flick.setPosition(0.1);

                    //right
                    if(targetColumn == 3){
                        encoderTarget = 0;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        encoderTarget = 500;
                    }
                    //left
                    else if(targetColumn == 1){
                        encoderTarget = 1000;
                    }

                    case_switch = 203;

                    break;

                case 203:

                    CameraDevice.getInstance().setFlashTorchMode(false);

                    //turn to 90 degrees + retract servo


                    gyroTurn(-90, 2);

                    if(heading > -92 && heading < -88){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 204;
                    }

                    break;

                case 204:

                    //drive forward

                    driveToTarget(1700, 0.3, 0.07);

                    if(1680 < -currentpos && -currentpos < 1720){
                        Drive(0);
                        case_switch = 205;
                    }

                    break;

                case 205:

                    //turn 90 deg to right

                    gyroTurn(0, 2);

                    if(heading > -2 && heading < 2){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 206;
                    }

                    break;

                case 206:

                    //drive to correct column

                    driveToTarget(encoderTarget, 0.3, 0.07);

                    if((encoderTarget - 20 < -currentpos && -currentpos < (encoderTarget + 20))){
                        Drive(0);
                        runtime.reset();
                        intaketimer.reset();
                        case_switch = 207;
                    }

                    else if(encoderTarget < -currentpos){
                        Drive(0);
                        runtime.reset();
                        intaketimer.reset();
                        case_switch = 207;
                    }


                    break;

                case 207:

                    //stop, turn towards cryptobox

                    depleft.setPosition(.55);

                    gyroTurn(110, 2);

                    if(heading < 112 && heading > 108){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 208;
                    }

                    break;

                case 208:

                    case_switch = 209;

                case 209:


                    //lift glyph bed, back up specific encoder distance, release grips

                    flip.setPosition(0.865);

                    //flipL.setPosition(0.81);
                   // flipR.setPosition(0.19);

                    encoderTarget = -200;
                    driveToTarget(encoderTarget, 0.15, 0.1);

                    Drive(-0.2);

                    if(encoderTarget - 100 < -currentpos && -currentpos < encoderTarget + 100){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.5);
                        runtime.reset();
                        case_switch = 210;
                    }

                    break;

                case 210:

                    //drive forward and back a few times to knock glyph into position

                    flip.setPosition(0.25);

                   // flipL.setPosition(0.2);
                  //  flipR.setPosition(0.8);

                    Drive(0.15);
                    sleep(500);
                    Drive(-0.15);
                    sleep(1200);
                    Drive(0.15);
                    sleep(1000);

                    case_switch = 211;

                    break;

                case 211:

                    //stop
                    Drive(0);

                    break;


                /*BLUE POSITION 2******************************************************************/


                case 300:

                    CameraDevice.getInstance().setFlashTorchMode(true);

                    //extend jewel arm

                    flick.setPosition(0.45);

                    sleep(100);

                    jewel.setPosition(0.2);

                    case_switch = 301;

                    break;

                case 301:

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
                        jewel.setPosition(1);
                        case_switch = 302;
                    }



                    break;


                case 302:

                    //pivot to target angle and read vuforia dab on em XD

                    jewel.setPosition(1);
                    flick.setPosition(0.1);

                    //left
                    if(targetColumn == 1){
                        encoderTarget = 0;
                    }
                    //center or unknown
                    else if(targetColumn == 2 || targetColumn == 0){
                        encoderTarget = 450;
                    }
                    //right
                    else if(targetColumn == 3){
                        encoderTarget = 1000;
                    }

                    case_switch = 303;

                    break;

                case 303:

                    CameraDevice.getInstance().setFlashTorchMode(false);

                    //turn to 90 degrees + retract servo


                    gyroTurn(90, 2);

                    if(heading > 88 && heading < 92){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 304;
                    }

                    break;

                case 304:

                    //drive forward

                    driveToTarget(1700, 0.3, 0.07);

                    if(1680 < -currentpos && -currentpos < 1720){
                        Drive(0);
                        case_switch = 305;
                    }

                    break;

                case 305:

                    //turn 90 deg to right

                    gyroTurn(0, 2);

                    if(heading > -2 && heading < 2){
                        Drive(0);
                        resetDriveEncoders();
                        case_switch = 306;
                    }

                    break;

                case 306:

                    //drive to correct column

                    driveToTarget(encoderTarget, 0.3, 0.07);

                    if((encoderTarget - 20 < -currentpos && -currentpos < (encoderTarget + 20))){
                        Drive(0);
                        runtime.reset();
                        intaketimer.reset();
                        case_switch = 307;
                    }

                    else if(encoderTarget < -currentpos){
                        Drive(0);
                        runtime.reset();
                        intaketimer.reset();
                        case_switch = 207;
                    }


                    break;

                case 307:

                    //stop, turn towards cryptobox

                    depleft.setPosition(.55);


                    gyroTurn(-110, 2);

                    if(heading < -108 && heading > -112){
                        Drive(0);
                        resetDriveEncoders();
                        flip.setPosition(0.865);

                       // flipL.setPosition(0.81);
                       // flipR.setPosition(0.19);
                        case_switch = 308;
                    }

                    break;

                case 308:

                    case_switch = 309;


                case 309:

                    //lift glyph bed, back up specific encoder distance, release grips

                    flip.setPosition(0.865);

                    // flipL.setPosition(0.81);
                   // flipR.setPosition(0.19);

                    encoderTarget = -400;
                    driveToTarget(encoderTarget, 0.1, 0.1);

                    Drive(-0.2);

                    if(encoderTarget - 100 < -currentpos && -currentpos < encoderTarget + 100){
                        Drive(0);
                        //open grips
                        grip.setPosition(0.5);
                        runtime.reset();
                        case_switch = 310;
                    }


                    break;

                case 310:

                    //drive forward and back a few times to knock glyph into position

                    flip.setPosition(0.25);


                    // flipL.setPosition(0.2);
                   // flipR.setPosition(0.8);

                    Drive(0.15);
                    sleep(500);
                    Drive(-0.15);
                    sleep(1200);
                    Drive(0.15);
                    sleep(1000);

                    case_switch = 311;

                    break;

                case 311:

                    //stop
                    Drive(0);

                    break;
            }


            telemetry.addData("JewelPos", jewel.getPosition());

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

        //if overshoot, drive backwards
      /*  else if(-currentpos > target){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }*/


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

        double baseTurnPowerFactor = 0.006;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.05;

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

