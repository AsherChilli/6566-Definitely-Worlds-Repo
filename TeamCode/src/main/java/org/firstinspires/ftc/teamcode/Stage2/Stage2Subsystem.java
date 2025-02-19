package org.firstinspires.ftc.teamcode.Stage2;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//import com.qualcomm.robotcore.hardware.Servo;

public class Stage2Subsystem extends SubsystemBase {

    private static DcMotorEx AngleMotor;
    //private static DcMotorEx extenderMotorRight = null;

    private static ElapsedTime runtime = new ElapsedTime();
    private static double time = 0;
    private static Servo clipHold;
    private static Servo clipWrist;

    private static Servo clipServo;

    private static Servo clipRackLeft;
    private static Servo clipRackRight;

    private int stage2State = 0;

    public final int pickFromRack = 1000;
    public final int readyPickFromRack = 900;
    public final int readyClipVal = 1100;
    public final int clipVal = 1200;

    private Timer stage2timer = new Timer();



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
    private static double rackPos;

    private static double clipServoPos = 0.5;


    private static final int angMin = 0;
    private static final int angMax = (int) 55 + 1493;



//    private static final PIDController extendController = new PIDController(pExtend, iExtend, dExtend);


    private static boolean isBusy = false;

    public Stage2Subsystem(final HardwareMap hMap){

        AngleMotor = hMap.get(DcMotorEx.class, "CAR");

        clipHold = hMap.get(Servo.class, "CLH");
        clipWrist = hMap.get(Servo.class, "CWR");

        clipServo = hMap.get(Servo.class, "CS");

        clipRackLeft = hMap.get(Servo.class, "CCL");
        clipRackRight = hMap.get(Servo.class, "CCR");





        AngleMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        AngleMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        AngleMotor.setTargetPosition(0);
        AngleMotor.setPower(.5);
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

    public static void setClipServoPos(double pos) {clipServoPos = pos;}
    public static double getClipServoPos() {return clipServoPos;}

    public static boolean isBusy(){return isBusy;}



    public static void setAngPower(double power){angPower = power;}

    public static void holdOpenMax() {clawPos = 0.475;}
    public static void holdClose() {clawPos = 0.35;}
    public static void holdCloseTight() {clawPos = 0.3;}

    public void raiseCams(){


        rackPos = 0;

    }
    public void lowerCams(){

        rackPos = 0.7;



    }

    //Make sure everything for getting a clip is in the right place
    public void readyClipRack() {
        lowerCams();
        //setClawPos(0.5);
        setAngTarget(-670 + 1493);
        setAngPower(.7);
    }
    //Get the clip from the clip rack
    public void clipRack() {
        //Grab clip in thingy
        holdOpenMax();
        setClawWristPos(.65);
    }
    public void clipRack2() {
        setAngTarget(1493);
        setAngPower(.3);
    }
    public void clipRack3(){
        setClawWristPos(.5);
    }
    public void clipRack4() {
        holdClose();
    }
    public void clipRack5() {
        setClawWristPos(.6);
        holdCloseTight();
    }
    public void clipRack6() {
        setAngTarget(-300 + 1493);
        setAngPower(.4);
    }
    public void readyClip(){
        setAngTarget(-300 + 1493);
        setClawWristPos(.925);
    }
    public void readyClip2() {
        setAngTarget(55 + 1493);
        setClawWristPos(.81);
    }
    public void readyClip3() {
        setClawWristPos(.825);
    }
    public void clip() {
        holdCloseTight();
        setClawWristPos(1);
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
        clawWristPos = Math.max( 0, Math.min(1, clawWristPos));
        rackPos = Math.max(0,Math.min(0.7, rackPos));
        clipServoPos = Math.max(0, Math.min(1, clipServoPos));


        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
//        extendPower = Math.max(-1, Math.min(1, extendController.calculate(AngleMotor.getCurrentPosition(), angTarget)));
        clipHold.setPosition(clawPos);
        clipWrist.setPosition(clawWristPos);
        AngleMotor.setTargetPosition(angTarget);

        clipServo.setPosition(clipServoPos);

        clipRackLeft.setPosition(rackPos);
        clipRackRight.setPosition(1.2-rackPos);

        AngleMotor.setPower(angPower);
        isBusy = !( angPos >= angTarget - 10 && angPos <= angTarget + 10);
    }


    public void setStage2(int status) {
        stage2State = status;
        stage2timer.resetTimer();
    }
    public void stage2Updater() {
        switch (stage2State) {
            case 0:
                break;
            case 1:
                break;
            case readyPickFromRack:
                if (stage2timer.getElapsedTime() < 1000) readyClipRack();
                else setStage2(0);
                break;
            case pickFromRack:
                if (stage2timer.getElapsedTime() < 1000) {
                    clipRack();
                } else if (stage2timer.getElapsedTime() < 2000) {
                    clipRack2();
                } else if (stage2timer.getElapsedTime() < 3000) {
                    clipRack3();
                } else if (stage2timer.getElapsedTime() < 4000) {
                    clipRack4();
                } else if (stage2timer.getElapsedTime() < 5000) {
                    clipRack5();
                } else if (stage2timer.getElapsedTime() < 6000) {
                    clipRack6();
                } else setStage2(0);
                break;
            case readyClipVal:
                if (stage2timer.getElapsedTime() < 1000) readyClip();
                else if (stage2timer.getElapsedTime() < 2000) readyClip2();
                //else if (stage2timer.getElapsedTime() < 3000) readyClip3();
                else setStage2(0);
                break;
            case clipVal:
                if (stage2timer.getElapsedTime() < 1000) clip();
                else setStage2(0);
                break;


//            case 1010:
//                Stage2Subsystem.setAngTarget(-670 + 1493);
//                Stage2Subsystem.setAngPower(0.7);
//                Stage2Subsystem.setClawPos(0);
//                if (Stage2Subsystem.getAngPos() == -670 + 1493 && stage2timer.getElapsedTime() > 1500) {
//                    Stage2Subsystem.setClawPos(0.5);
//                    Stage2Subsystem.setAngTarget(-600 + 1493);
//                } else if (stage2timer.getElapsedTime() > 2500) {
//                    setStage2(1001);
//                }
//                break;
//
//
//            case 1001:
//                Stage2Subsystem.holdOpenMax();
//                Stage2Subsystem.setClawWristPos(0.65);
//
//
//                break;
//            case 1002:
//                Stage2Subsystem.setClawWristPos(0.65);
//                Stage2Subsystem.holdOpenMax();
//                Stage2Subsystem.setAngTarget(0 + 1493);
//                Stage2Subsystem.setAngPower(0.3);
//                break;
//
//            case 1003:
//                Stage2Subsystem.setClawPos(.46);
//                Stage2Subsystem.setClawWristPos(.5);
//                if (stage2timer.getElapsedTime() > 250) {
//                    Stage2Subsystem.holdClose();
//                }
//                break;
//            case 1004:
//                Stage2Subsystem.setClawWristPos(.6);
//                Stage2Subsystem.holdCloseTight();
//                if (stage2timer.getElapsedTime() > 250) {
//                    Stage2Subsystem.setAngTarget(-300 + 1493);
//                    Stage2Subsystem.setAngPower(.4);
//                }
//                break;
//            case 1005:
//                Stage2Subsystem.setClawWristPos(.825);
//                break;
//            case 1006:
//                Stage2Subsystem.setAngTarget(55 + 1493);
//                break;
        }
    }

}

