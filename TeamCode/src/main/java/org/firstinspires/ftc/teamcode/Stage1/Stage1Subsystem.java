package org.firstinspires.ftc.teamcode.Stage1;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Servo;

public class Stage1Subsystem extends SubsystemBase {

    private static DcMotorEx extenderMotorLeft = null;
    private static DcMotorEx extenderMotorRight = null;

    private static Servo clawServo = null;
    private static Servo clawWristServo = null;
    private static Servo clawTwistServo = null;

    private static ColorSensor colorSensor = null;






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




        extenderMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extenderMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extenderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        extenderMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        extendController.setPID(pExtend, iExtend, dExtend);

    }


    // ----------------
    // Setters
    // ----------------
    public static void setPos(double ext) {

        extTarget = (int) ext;
    }

    public static void setPosIN(double ext) {
        extTarget = (int) (ext * ticks_in_inch);
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
    public static double getClawAnglePos(){return clawTwistPos;}
    public static double getRed(){ return  colorSensor.red();}
    public static double getBlue(){return colorSensor.blue();};
    public static double getGreen(){return colorSensor.green();}

    public static void setClawPos(double pos) {
        clawPos = pos;
    }
    public static void setClawWristPos(double pos) {
        clawWristPos = pos;
    }
    public static void setClawAnglePos(double pos) {
        clawTwistPos = pos;
    }

    public static boolean isBusy(){return isBusy;}

    public static void tuck() {
        extTarget = 0;
        clawWristPos = 0.7;
        clawTwistPos = 0.5;
    }
    public static void close() {setClawPos(0.42);}
    public static void closeTight() {setClawPos(0.28);}
    public static void open() {setClawPos(0.6);}


    public static void setExtendPID(double pExtend, double iExtend, double dExtend){
        extendController.setPID(pExtend, iExtend, dExtend);
    }

    public static double getExtendP() {return pExtend;}
    public static double getExtendI() {return iExtend;}
    public static double getExtendD() {return dExtend;}

    // ----------------
    // Calculations
    // ----------------



    public static void update() {
        double extendPower;
        extPos = extenderMotorLeft.getCurrentPosition();






        // CLamping

        extTarget =  Math.max(extMin, Math.min(extMax, extTarget));
        clawPos = Math.max(0, Math.min(1, clawPos));
        clawTwistPos = Math.max(0, Math.min(1, clawTwistPos));
        clawWristPos = Math.max(0.4, Math.min(extPos < 400 ? 0.7 : 1, clawWristPos));


        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
        extendPower = Math.max(-.8, Math.min(.8, extendController.calculate(extenderMotorLeft.getCurrentPosition(), extTarget)));

        clawServo.setPosition(clawPos);
        clawWristServo.setPosition(clawWristPos);
        clawTwistServo.setPosition(clawTwistPos);

        extenderMotorLeft.setPower(extendPower);
        extenderMotorRight.setPower(extendPower);
        isBusy = !( extPos >= extTarget - 10 && extPos <= extTarget + 10);
    }
}
