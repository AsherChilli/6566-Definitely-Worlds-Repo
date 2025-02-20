package org.firstinspires.ftc.teamcode.Stage1;



import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.Processors.sampleProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.RotatedRect;
//import com.qualcomm.robotcore.hardware.Servo;

public class Stage1Subsystem extends SubsystemBase {

    private static DcMotorEx extenderMotorLeft = null;
    private static DcMotorEx extenderMotorRight = null;

    private static Servo clawServo = null;
    private static Servo clawWristServo = null;
    private static Servo clawTwistServo = null;

    private static ColorSensor colorSensor = null;
    private static DistanceSensor distanceSensor = null;


    private static int stage1State = 0;

    private static Timer stage1Timer = new Timer();

    static VisionPortal visionPortal;

    sampleProcessor processor = new sampleProcessor();

    //Some variable for color
    static sampleProcessor.Color color = sampleProcessor.Color.RED;



    static double ticks_per_rotation = 751.8;
    //TODO: UPDATE THIS VIA EMPIRICAL DATA 96 deg
    static double gear_reduction = 1/(800/((ticks_per_rotation * 360) / 96));

    private static final double ticks_in_degree = (ticks_per_rotation * gear_reduction) / 360;


    //Extension Motor
    static double ticks_per_rotation_ext = 537.7;

    private static final double ticks_in_mm = ticks_per_rotation_ext / 120;
    private static final double ticks_in_inch = ticks_in_mm / 25.4;


    private static double pExtend = 0.01, iExtend = 0/*0.05*/, dExtend = 0.0004, fExtend = 0;



    private static int extPos = 0;
    private static int extTarget = 0;
    private static boolean pidMoving = false;
    private static double extPow = 0;

    private static double clawPos = 0.4;
    private static double clawWristPos = 0.6;
    private static double clawTwistPos = 0.625;


    private static final int extMin = 0;
    private static final int extMax = 2500;//(int) (ticks_in_inch * 42)-22;



    private static final PIDController extendController = new PIDController(pExtend, iExtend, dExtend);


    private static boolean isBusy = false;

    public Stage1Subsystem(final HardwareMap hMap){

        extenderMotorLeft = hMap.get(DcMotorEx.class, "EXL");
        extenderMotorRight = hMap.get(DcMotorEx.class,  "EXR");

        clawServo = hMap.get(Servo.class, "GMC");
        clawWristServo = hMap.get(Servo.class, "GMW");
        clawTwistServo = hMap.get(Servo.class, "GMT");

        colorSensor = hMap.get(ColorSensor.class, "ColorSensor");
        distanceSensor = hMap.get(DistanceSensor.class, "ColorSensor");




        extenderMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extenderMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extenderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        extenderMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        extendController.setPID(pExtend, iExtend, dExtend);



        visionPortal = new VisionPortal.Builder()
                //Get camera from hMap
                .setCamera(hMap.get(WebcamName.class, "Webcam 1"))
                //Add all needed processors (hand written)
                .addProcessor(processor)
                //Set camera resolution
                .setCameraResolution(new Size(640, 480))
                //No idea
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //Enable streaming to Dhub and FTC Dashboard (?)
                //TODO: Set to false to reduce cpu usage
                .enableLiveView(true)
                //If all proccesors disabled, stop streaming
                .setAutoStopLiveView(true)
                //finish
                .build();

    }


    // ----------------
    // Setters
    // ----------------
    public static void setPos(double ext) {extTarget = (int) ext; pidMoving = true;}
    public static void setPosIN(double ext) {extTarget = (int) (ext * ticks_in_inch); pidMoving = true;}
    //Shattuck's slide interpretation
    public static void setExtPow(double pow) {
        extPow = pow;
        if (extPow != 0) {pidMoving = false;}
    }
    // ----------------
    // Getters
    // ----------------

    public static int getExtTarget(){return extTarget;}
    public double getExtTargetIN(){return (extTarget / ticks_in_inch);}

    public static int getExtenderPos(){return extenderMotorLeft.getCurrentPosition();}
    public static double getExtenderPosIN(){return (extenderMotorLeft.getCurrentPosition() / ticks_in_inch);}

    public static double getClawPos(){return clawPos;}
    public static double getClawWristPos(){return clawWristPos;}
    public static double getClawTwistPos(){return clawTwistPos;}
    public static double getRed(){ return  colorSensor.red();}
    public static double getBlue(){return colorSensor.blue();};
    public static double getGreen(){return colorSensor.green();}
    public static double getAlpha(){return colorSensor.alpha();}
    public static double getDistance(){return distanceSensor.getDistance(DistanceUnit.INCH);};

    public static void setClawPos(double pos) {
        clawPos = pos;
    }
    public static void setClawWristPos(double pos) {
        clawWristPos = pos;
    }
    public static void setClawTwistPos(double pos) {
        clawTwistPos = pos;
    }

    public static boolean isBusy(){return isBusy;}

    public static void tuck() {
        extTarget = 0;
        clawWristPos = 0.7;
        clawTwistPos = 0.5;
    }
    public static void close() {setClawPos(0.455);} // .42 previously
    public static void closeTight() {setClawPos(0.28);}
    public static void open() {setClawPos(0.6);}


    public static void up() {setClawWristPos(0.65);}
    public static void down() {setClawWristPos(1);}

    public static void setRed() {color = sampleProcessor.Color.RED;}
    public static void setBlue() {color = sampleProcessor.Color.BLUE;}

    public static sampleProcessor.Color getColor() {
        return color;
    }

    public static void stopStream() {visionPortal.stopStreaming();}
    public static void startStream() {visionPortal.resumeStreaming();}

    public static void setExtendPID(double pExtend, double iExtend, double dExtend){
        extendController.setPID(pExtend, iExtend, dExtend);
    }

    public static double getExtendP() {return pExtend;}
    public static double getExtendI() {return iExtend;}
    public static double getExtendD() {return dExtend;}

    // ----------------
    // Calculations
    // ----------------


    public static void pickupSample() {
        //RotatedRect rect = sampleProcessor.getRect();

        if (getRed() > 285 && getBlue() < 403 && getDistance() < 1.32) {
            close();
        }
        if (sampleProcessor.getWidth() > 450) {
            setClawTwistPos(0);
        } else if (sampleProcessor.getWidth() < 450) setClawTwistPos(0.625);
    }


    public static void update() {
        double extendPower;
        extPos = extenderMotorLeft.getCurrentPosition();

        sampleProcessor.setColor(color);






        // Clamping

        extTarget =  Math.max(extMin, Math.min(extMax, extTarget));
        clawPos = Math.max(0, Math.min(1, clawPos));
        clawTwistPos = Math.max(0, Math.min(1, clawTwistPos));
        clawWristPos = Math.max(0.4, Math.min(extPos < 400 ? 0.7 : 1, clawWristPos));
        if (extPos < 400) { clawTwistPos = .625;}


        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
        extendPower = Math.max(-.8, Math.min(.8, extendController.calculate(extenderMotorLeft.getCurrentPosition(), extTarget)));

        clawServo.setPosition(clawPos);
        clawWristServo.setPosition(clawWristPos);
        clawTwistServo.setPosition(clawTwistPos);

        if (pidMoving) {
            extenderMotorLeft.setPower(extendPower);
            extenderMotorRight.setPower(extendPower);
            isBusy = !(extPos >= extTarget - 10 && extPos <= extTarget + 10);
        }
        else {
            //ARM LIMITS
            if (!(extPos < extMin && extPow < 0) && !(extPos > extMax && extPow > 0)) {
                extenderMotorLeft.setPower(extPow);
                extenderMotorRight.setPower(extPow);
            }

            //Determines if busy
            if (extPow != 0) {isBusy = true;}
            else {isBusy = false;}
        }
    }

    public static void setStage1(int status) {
        stage1State = status;
        stage1Timer.resetTimer();
    }
    public static void stage1Updater() {
        switch (stage1State) {
            case 0:
                break;
            case 1:
                break;

        }
    }
}
