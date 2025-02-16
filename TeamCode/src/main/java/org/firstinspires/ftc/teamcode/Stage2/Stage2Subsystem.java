package org.firstinspires.ftc.teamcode.Stage2;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stage1.Tuning.ArmTuner;
//import com.qualcomm.robotcore.hardware.Servo;

public class Stage2Subsystem extends SubsystemBase {

    private static DcMotorEx AngleMotor;
    //private static DcMotorEx extenderMotorRight = null;

    private static ElapsedTime runtime = new ElapsedTime();
    private static double time = 0;
    private static Servo clipHold;
    private static Servo clipWrist;

    private static Servo clipRackLeft;
    private static Servo clipRackRight;



    static double ticks_per_rotation = 751.8;
    //TODO: UPDATE THIS VIA EMPIRICAL DATA 96 deg
    static double gear_reduction = 1/(800/((ticks_per_rotation * 360) / 96));

    private static final double ticks_in_degree = (ticks_per_rotation * gear_reduction) / 360;


    //Extension Motor
    static double ticks_per_rotation_ext = 537.7;

    private static final double ticks_in_mm = ticks_per_rotation_ext / 120;
    private static final double ticks_in_inch = ticks_in_mm / 25.4;



//    private static final double pAngle = 0.008, iAngle = 0.0, dAngle = 0.0008;
//    private static double fAngle = 0.1;



    private static int angPos = 0;
    private static int angTarget = 0;
    private static double angPower = 0;

    private static double clawPos;
    private static double clawWristPos;


    private static final int angMin = 0;
    private static final int angMax = (int) 55 + 1493;



//    private static final PIDController extendController = new PIDController(pExtend, iExtend, dExtend);


    private static boolean isBusy = false;

    public Stage2Subsystem(final HardwareMap hMap){

        AngleMotor = hMap.get(DcMotorEx.class, "CAR");

        clipHold = hMap.get(Servo.class, "CWR");
        clipWrist = hMap.get(Servo.class, "CLH");

        clipRackLeft = hMap.get(Servo.class, "CCL");
        clipRackRight = hMap.get(Servo.class, "CCR");





        AngleMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        AngleMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        AngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);







//        extendController.setPID(pExtend, iExtend, dExtend);

    }


    // ----------------
    // Setters
    // ----------------
    public static void setAngTarget(double ang) {angTarget = (int) ang;}

    // ----------------
    // Getters
    // ----------------

    public static int getAngTarget(){return angTarget;}
    public double getAngTargetIN(){return (angTarget / ticks_in_inch);}



    public static int getAngPos(){return AngleMotor.getCurrentPosition();}
    public static double getAngPosIN(){return (AngleMotor.getCurrentPosition() / ticks_in_inch);}

    public static double getClawPos(){return clawPos;}
    public static double getClawWristPos(){return clawWristPos;}

    public static void setClawPos(double pos) {clawPos = pos;}
    public static void setClawWristPos(double pos) {clawWristPos = pos;}

    public static boolean isBusy(){return isBusy;}



    public static void setAngPower(double power){angPower = power;}

    public static void holdOpenMax() {clawPos = 0.475;}
    public static void holdClose() {clawPos = 0.35;}
    public static void holdCloseTight() {clawPos = 0.3;}

    public void raiseCams(){

        clipRackLeft.setPosition(.7);
        clipRackRight.setPosition(.7);

    }
    public void lowerCams(){

        clipRackLeft.setPosition(.4);
        clipRackRight.setPosition(.4);

    }

    //Make sure everything for getting a clip is in the right place
    public void readyClipRack() {
        lowerCams();
        clipHold.setPosition(0.475);
        AngleMotor.setTargetPosition(-670 + 1493);
        AngleMotor.setPower(.7);
        AngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Get the clip from the clip rack
    public void clipRack() {
        //Grab clip in thingy
        clipWrist.setPosition(.6);
        time = runtime.time();
        while (runtime.time() - time <= .25) {
            continue;
        } if (runtime.time() - time > .25) {
            AngleMotor.setTargetPosition(-300 + 1493);
            AngleMotor.setPower(.4);
        } else if(runtime.time() - time > .75){
            clipHold.setPosition(.3);
        }
    }
    public void readyClip() {



    }
    public void clip() {

    }
    public void readyScore() {

    }
    public void score() {

    }

//    public static void setExtendPID(double pExtend, double iExtend, double dExtend){
//        extendController.setPID(pExtend, iExtend, dExtend);
//    }

//    public static double getExtendP() {return pExtend;}
//    public static double getExtendI() {return iExtend;}
//    public static double getExtendD() {return dExtend;}

    // ----------------
    // Calculations
    // ----------------



    public static void update() {
        // CLamping

        angTarget =  Math.max(angMin, Math.min(angMax, angTarget));
        clawPos = Math.max(0, Math.min(1, clawPos));
        clawWristPos = Math.max(angPos < 200 ? 0.7 : 0, Math.min(1, clawWristPos));


        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
//        extendPower = Math.max(-1, Math.min(1, extendController.calculate(AngleMotor.getCurrentPosition(), angTarget)));
        clipHold.setPosition(clawPos);
        clipWrist.setPosition(clawWristPos);

        AngleMotor.setPower(angPower);
        isBusy = !( angPos >= angTarget - 10 && angPos <= angTarget + 10);
    }
}
