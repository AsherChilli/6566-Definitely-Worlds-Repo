package org.firstinspires.ftc.teamcode.new6566;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="UFODrive", group = "TeleOp")
public class UFODrive extends OpMode {
    UFODriveHardware r = new UFODriveHardware();

    /*
     * VARIABLES & CONSTANTS
     */


    @Override
    public void init() {
        r.Left.setPower(0.0);
        r.Right.setPower(0.0);
        r.Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        r.Left.setPower(gamepad1.left_stick_y *.5);
        r.Right.setPower(gamepad1.left_stick_x *.5);
    }
}