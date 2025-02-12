package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class SleekClippaHardware {
    OpMode opMode;
    public DcMotor TopLeft, TopRight, BottomLeft, BottomRight , Elbow, ChainMotor,ExtendRight,ExtendLeft;
    //public DcMotor[] drive;
    public DcMotorEx ClipElbow;
    public DcMotor ClipArm;



    public Servo ClawFingers,ClawWrist,ClipHold, Rack, Tilter,ClipCamLeft,ClipCamRight;
    public Servo ClipWrist, GameWrist, GameTwist, GameClaw;
    public AnalogInput ranger;
    public ModernRoboticsI2cColorSensor ArmColorSensor;





    public void init_robot(OpMode opMode){

        this.opMode = opMode;

        init_hardware();

    }

    public boolean getTolerance(double val1, double val2, double tolerance){
        return (val1+tolerance >val2) && (val1-tolerance < val2);
    }
    public void init_hardware(){

        //ArmColorSensor = (ModernRoboticsI2cColorSensor) opMode.hardwareMap.colorSensor.get("ACS");
        TopLeft = opMode.hardwareMap.dcMotor.get("TLM");
        TopRight = opMode.hardwareMap.dcMotor.get("TRM");
        BottomLeft = opMode.hardwareMap.dcMotor.get("BLM");
        BottomRight = opMode.hardwareMap.dcMotor.get("BRM");

//        ChainMotor = opMode.hardwareMap.dcMotor.get("CHM");
//        Elbow = opMode.hardwareMap.dcMotor.get("ELB"); //Motor furthest from the body of the bot

        ClipCamRight = opMode.hardwareMap.servo.get("CCR");
        ClipCamLeft = opMode.hardwareMap.servo.get("CCL");
        ExtendLeft = opMode.hardwareMap.dcMotor.get("EXL");
        ExtendRight = opMode.hardwareMap.dcMotor.get("EXR");
        ClipArm = opMode.hardwareMap.get(DcMotorEx.class, "CAR");
        //ClipArm = opMode.hardwareMap.dcMotor.get("CAR"); // Clip Elbow
        ClipWrist = opMode.hardwareMap.servo.get("CWR"); //Rotates the Grabber not the wrist
        ClipHold = opMode.hardwareMap.servo.get("CLH"); //Actually pinches the clip


        //DcMotorEx clipElbowEx = (DcMotorEx) hardwareMap.dcMotor.get(DcMotor.class, "clipElbow");
        //ClipElbow = opMode.hardwareMap.get(DcMotorEx.class, "CEB");



       // ranger = opMode.hardwareMap.get(AnalogInput.class, "ranger");



//        Tilter = opMode.hardwareMap.servo.get("TLT");
       // Rack = opMode.hardwareMap.servo.get("RCK");

        GameWrist = opMode.hardwareMap.servo.get("GMW"); //wrist for the game element claw
        GameTwist = opMode.hardwareMap.servo.get("GMT"); //Twister for the game element claw
        GameClaw = opMode.hardwareMap.servo.get("GMC"); //Claw for the game element claw


        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopRight.setDirection(DcMotorSimple.Direction.FORWARD);
        opMode.telemetry.addData("ExtendRight Position", ExtendRight.getCurrentPosition());
        opMode.telemetry.addData("ExtendLeft Position", ExtendLeft.getCurrentPosition());
        opMode.telemetry.update();
//        }
    }
}
