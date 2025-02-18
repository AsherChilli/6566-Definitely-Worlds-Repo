package org.firstinspires.ftc.teamcode.new6566;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name="Awesome Drive", group = "TeleOp")
public class SleekClippaDrive2 extends OpMode {
    SleekClippaHardware r = new SleekClippaHardware();
    private ElapsedTime runtime = new ElapsedTime();

    Stage1Subsystem stage1;
    Stage2Subsystem stage2;

    //Clip Vars
    double holdOpenMax = .475;//.45
    double holdClose = .35;//.3
    double holdCloseTight = .3;//.2
    boolean clipping = false;

    Follower follower;


    @Override
    public void init() {
        //r.init_robot(this);
        follower = new Follower(hardwareMap);
        stage2 = new Stage2Subsystem(hardwareMap);
        stage1 = new Stage1Subsystem(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower.startTeleopDrive();
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

        Stage1Subsystem.setPos(Stage1Subsystem.getExtTarget() + gamepad1.right_trigger * 20 + gamepad1.left_trigger * -20);

        if (gamepad1.left_bumper) Stage1Subsystem.close();
        else if (gamepad1.right_bumper) Stage1Subsystem.open();

        if (gamepad2.dpad_up) Stage2Subsystem.setAngTarget(Stage2Subsystem.getAngTarget() + 10);
        else if (gamepad2.dpad_down) Stage2Subsystem.setAngTarget(Stage2Subsystem.getAngTarget() - 10);


        if(gamepad2.triangle){stage2.setStage2(stage2.readyPickFromRack);}
        if(gamepad2.square){stage2.setStage2(stage2.pickFromRack);}
        if(gamepad2.cross){stage2.setStage2(stage2.readyClipVal);}
        if (gamepad2.circle) {
            stage2.setStage2(stage2.clipVal);
            Stage1Subsystem.closeTight();
            Stage1Subsystem.setClawWristPos(0);

        }
        if(gamepad2.dpad_left){stage2.raiseCams();}
        else stage2.lowerCams();

        if(gamepad1.dpad_right) {Stage1Subsystem.setClawWristPos(Stage1Subsystem.getClawWristPos() + 0.01);}
        else if (gamepad1.dpad_left) {Stage1Subsystem.setClawWristPos(Stage1Subsystem.getClawWristPos() - 0.01);}
        if (gamepad1.dpad_up) {Stage1Subsystem.setClawAnglePos(Stage1Subsystem.getClawAnglePos() + 0.01);}
        else if (gamepad1.dpad_down) {Stage1Subsystem.setClawAnglePos(Stage1Subsystem.getClawAnglePos() - 0.01);}

//        if (gamepad2.a) Stage2Subsystem.holdClose();
//        else if (gamepad2.b) Stage2Subsystem.holdOpenMax();



        Stage1Subsystem.update();
        Stage2Subsystem.update();
        stage2.stage2Updater();
        follower.updatePose();



//        telemetry.addData("Ext Pos", Stage1Subsystem.getExtenderPos());
//        telemetry.addData("Ext Target", Stage1Subsystem.getExtTarget());

//        telemetry.addLine("Arm Position: " + String.valueOf(r.ClipArm.getCurrentPosition()) );
//        telemetry.addLine("Extendo Position: " + String.valueOf(r.ExtendLeft.getCurrentPosition()) );
//        telemetry.addLine("Twist Position: " + String.valueOf(r.GameTwist.getPosition()) );

    }

}
