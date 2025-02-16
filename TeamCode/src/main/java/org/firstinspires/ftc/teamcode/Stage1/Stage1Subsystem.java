package org.firstinspires.ftc.teamcode.Stage1;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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






    static double ticks_per_rotation = 751.8;
    //TODO: UPDATE THIS VIA EMPIRICAL DATA 96 deg
    static double gear_reduction = 1/(800/((ticks_per_rotation * 360) / 96));

    private static final double ticks_in_degree = (ticks_per_rotation * gear_reduction) / 360;


    //Extension Motor
    static double ticks_per_rotation_ext = 537.7;

    private static final double ticks_in_mm = ticks_per_rotation_ext / 120;
    private static final double ticks_in_inch = ticks_in_mm / 25.4;


    private static double pExtend = 0.012, iExtend = 0/*0.05*/, dExtend = 0.0004, fExtend = 0;



    private static int extPos = 0;
    private static int extTarget = 0;

    private static double clawPos;
    private static double clawWristPos;
    private static double clawTwistPos;


    private static final int extMin = 0;
    private static final int extMax = (int) (ticks_in_inch * 42)-22;



    private static final PIDController extendController = new PIDController(pExtend, iExtend, dExtend);


    private static boolean isBusy = false;

    public Stage1Subsystem(final HardwareMap hMap){

        extenderMotorLeft = hMap.get(DcMotorEx.class, "EXL");
        extenderMotorRight = hMap.get(DcMotorEx.class,  "EXR");

        clawServo = hMap.get(Servo.class, "GMC");
        clawWristServo = hMap.get(Servo.class, "GMW");
        clawTwistServo = hMap.get(Servo.class, "GMT");




        extenderMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extenderMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



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



        double armExt = extenderMotorLeft.getCurrentPosition();




        // CLamping

        extTarget =  Math.max(extMin, Math.min(extMax, extTarget));
        clawPos = Math.max(0, Math.min(1, clawPos));
        clawTwistPos = Math.max(0, Math.min(1, clawTwistPos));
        clawWristPos = Math.max(extPos < 200 ? 0.7 : 0, Math.min(1, clawWristPos));


        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
        extendPower = Math.max(-1, Math.min(1, extendController.calculate(extenderMotorLeft.getCurrentPosition(), extTarget)));

        clawServo.setPosition(clawPos);
        clawWristServo.setPosition(clawWristPos);
        clawTwistServo.setPosition(clawTwistPos);

        extenderMotorLeft.setPower(extendPower);
        isBusy = !( armExt >= extTarget - 10 && armExt <= extTarget + 10);
    }
}
