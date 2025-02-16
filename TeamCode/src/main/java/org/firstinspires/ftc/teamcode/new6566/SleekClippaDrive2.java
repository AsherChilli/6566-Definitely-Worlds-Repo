package org.firstinspires.ftc.teamcode.new6566;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name="SleekClippaDrive", group = "TeleOp")
public class SleekClippaDrive2 extends OpMode {
    SleekClippaHardware r = new SleekClippaHardware();
    private ElapsedTime runtime = new ElapsedTime();

    Stage1Subsystem stage1 = new Stage1Subsystem(hardwareMap);
    Stage2Subsystem stage2 = new Stage2Subsystem(hardwareMap);

    //Clip Vars
    double holdOpenMax = .475;//.45
    double holdClose = .35;//.3
    double holdCloseTight = .3;//.2
    boolean clipping = false;

    Follower follower;

    @Override
    public void init() {
        r.init_robot(this);
        follower = new Follower(hardwareMap);
   }

    @Override
    public void loop() {

        /*
         *
         *Gamepad 1
         *
         */

        // Keep existing drive code unchanged

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        //Extend or retract arm

        Stage1Subsystem.setPos(Stage1Subsystem.getExtenderPos() + gamepad1.right_trigger * -20 + gamepad1.left_trigger * 20);

        if (gamepad1.left_bumper) Stage1Subsystem.close();
        else if (gamepad1.right_bumper) Stage1Subsystem.open();

        



        telemetry.addLine("Arm Position: " + String.valueOf(r.ClipArm.getCurrentPosition()) );
        telemetry.addLine("Extendo Position: " + String.valueOf(r.ExtendRight.getCurrentPosition()) );
        telemetry.addLine("Twist Position: " + String.valueOf(r.GameTwist.getPosition()) );

    }
}
