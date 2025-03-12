package org.firstinspires.ftc.teamcode.new6566;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class UFODriveHardware {
    OpMode opMode;
    public DcMotor Left, Right;

    public void init_robot(OpMode opMode){

        this.opMode = opMode;

        init_hardware();

    }

    public void init_hardware(){

        //ArmColorSensor = (ModernRoboticsI2cColorSensor) opMode.hardwareMap.colorSensor.get("ACS");
        Left = opMode.hardwareMap.dcMotor.get("LEF");
        Right = opMode.hardwareMap.dcMotor.get("RIG");
    }
}