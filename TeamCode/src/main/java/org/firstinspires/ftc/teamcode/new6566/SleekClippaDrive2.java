package org.firstinspires.ftc.teamcode.new6566;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

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

    int state = 0;

    final int clip = 1000;

    Timer timer = new Timer();

    Follower follower;


    @Override
    public void init() {
        //r.init_robot(this);
        follower = new Follower(hardwareMap);
        stage2 = new Stage2Subsystem(hardwareMap);
        stage1 = new Stage1Subsystem(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower.startTeleopDrive();


        //Stage1Subsystem.setExtPosBlind(100,1,Stage1Subsystem.BlindfoldReset());
//        Stage2Subsystem.setAngTarget(1000);
//        Stage1Subsystem.close();
//        Stage1Subsystem.up();
//        Stage1Subsystem.setClawTwistPos(0.625);
//        Stage2Subsystem.setClipServoPos(.3);
//        Stage2Subsystem.setClawWristPos(0);


   }

   @Override
   public void init_loop() {
//       Stage2Subsystem.update();
//       Stage1Subsystem.update();

   }

    @Override
    public void loop() {

        /*
         *
         *Gamepad 1
         *
         */

        // Keep existing drive code unchanged
        if (Stage1Subsystem.touchSensor.isPressed()) Stage1Subsystem.resetOffset();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        //Extend or retract arm

        if (gamepad1.right_trigger > .05) {Stage1Subsystem.setExtPow(gamepad1.right_trigger);}
        else if (gamepad1.left_trigger > .05) {Stage1Subsystem.setExtPow(-1 * gamepad1.left_trigger);}
        else {Stage1Subsystem.setExtPow(0);}
        //Stage1Subsystem.setPos(Stage1Subsystem.getExtTarget() + gamepad1.right_trigger * 30 + gamepad1.left_trigger * -30);

        //Stage1Subsystem.pickupSample();

        if (gamepad1.left_bumper) Stage1Subsystem.open();
        else if (gamepad1.right_bumper) Stage1Subsystem.close();

        if (gamepad1.triangle) Stage1Subsystem.up();
        if (gamepad1.cross) Stage1Subsystem.down();
        if (gamepad1.square) Stage1Subsystem.setClawTwistPos(0);
        if (gamepad1.circle) Stage1Subsystem.setClawTwistPos(0.625);
        //if (gamepad1.circle) Stage1Subsystem.setPos(255);

        if(gamepad1.dpad_down) {Stage1Subsystem.setClawWristPos(Stage1Subsystem.getClawWristPos() + 0.01);}
        else if (gamepad1.dpad_up) {Stage1Subsystem.setClawWristPos(Stage1Subsystem.getClawWristPos() - 0.01);}
        if (gamepad1.dpad_left) {Stage1Subsystem.setClawTwistPos(Stage1Subsystem.getClawTwistPos() + 0.01);}
        else if (gamepad1.dpad_right) {Stage1Subsystem.setClawTwistPos(Stage1Subsystem.getClawTwistPos() - 0.01);}

//        if (gamepad1.cross) {
//            Stage1Subsystem.setPos(Stage1Subsystem.getExtTarget() + 10);
//
//        }

        Stage2Subsystem.setAngTarget(Stage2Subsystem.getAngTarget() + gamepad2.right_stick_y * 20);
//        if (gamepad2.dpad_up) Stage2Subsystem.setAngTarget(Stage2Subsystem.getAngTarget() + 10);
//        else if (gamepad2.dpad_down) Stage2Subsystem.setAngTarget(Stage2Subsystem.getAngTarget() - 10);

        //Stage2Subsystem.setClipServoPos(Stage2Subsystem.getClipServoPos() + gamepad2.left_stick_y* 0.01);
        Stage2Subsystem.setClawWristPos(Stage2Subsystem.getClawWristPos() + gamepad2.left_stick_y * 0.03);

//        if(gamepad2.triangle){
//            Stage2Subsystem.setStage2(Stage2Subsystem.readyPickFromRack);}
//        if(gamepad2.square){
//            Stage2Subsystem.setStage2(Stage2Subsystem.pickFromRack);}
//        if(gamepad2.cross){
//            Stage2Subsystem.setStage2(Stage2Subsystem.readyClipVal);}
//        if (gamepad2.circle) {
//            Stage2Subsystem.setStage2(Stage2Subsystem.clipVal);
//            Stage1Subsystem.closeTight();
//            Stage1Subsystem.setClawWristPos(0);
//
//        }


        //TODO: uncomment to test full clip with no input
        if(gamepad2.cross) {
            setState(clip);
        }
        if(gamepad2.triangle) {
            setState(0);
        }
        if(gamepad2.b) {
            setState(1006);
        }
        if(gamepad2.dpad_left){
            Stage2Subsystem.raiseCams();}
        else Stage2Subsystem.lowerCams();
        if (gamepad2.dpad_up){
            Stage2Subsystem.readyScore();
            telemetry.addLine("Angle: " + String.valueOf(Stage2Subsystem.getAngTarget()));
        }

        if (gamepad2.left_bumper) Stage2Subsystem.holdOpenMax();
        else if (gamepad2.right_bumper) Stage2Subsystem.holdClose();
//        if (gamepad2.a) Stage2Subsystem.holdClose();
//        else if (gamepad2.b) Stage2Subsystem.holdOpenMax();

//        Stage1Subsystem.orientToSample();
        //Stage1Subsystem.pickupSample();
        Stage1Subsystem.update();
        Stage2Subsystem.update();
        Stage2Subsystem.stage2Updater();
        follower.update();
        follower.updatePose();
        autonomousUpdate();



        telemetry.addData("Ext Pos", Stage1Subsystem.getExtenderPos());
        telemetry.addData("Ext Target", Stage1Subsystem.getExtTarget());
        telemetry.addData("Clip servo pos", Stage2Subsystem.getClipServoPos());
        //telemetry.addData("red", Stage1Subsystem.getRed());
        //telemetry.addData("blue", Stage1Subsystem.getBlue());
        //telemetry.addData("green", Stage1Subsystem.getGreen());
        //telemetry.addData("alpha", Stage1Subsystem.getAlpha());
        //telemetry.addData("distance", Stage1Subsystem.getDistance());

        telemetry.addData("Arm Pos", Stage2Subsystem.getAngTarget());

        //telemetry.addData("Claw Pos", Stage1Subsystem.getClawPos());
        //telemetry.addData("Claw Wrist Pos", Stage1Subsystem.getClawWristPos());
        //telemetry.addData("Claw Twist Pos", Stage1Subsystem.getClawTwistPos());

        telemetry.addData("Clip Claw Wrist", Stage2Subsystem.getClawWristPos());
        telemetry.addData("Clip Claw", Stage2Subsystem.getClawPos());
//        telemetry.addLine("Arm Position: " + String.valueOf(r.ClipArm.getCurrentPosition()) );
//        telemetry.addLine("Extendo Position: " + String.valueOf(r.ExtendLeft.getCurrentPosition()) );
//        telemetry.addLine("Twist Position: " + String.valueOf(r.GameTwist.getPosition()) );

    }

    void setState(int num) {
        state = num;
        timer.resetTimer();
    }

    void autonomousUpdate() {
        switch (state) {
            case 0:
                break;
            case clip:
                Stage2Subsystem.setStage2(Stage2Subsystem.readyPickFromRack);
                Stage1Subsystem.up();
                Stage1Subsystem.close();
                setState(1001);
                break;
            case 1001:
                if (timer.getElapsedTime() > 1800) {
                    setState(1002);
                }
                break;
            case 1002:
                Stage2Subsystem.setStage2(Stage2Subsystem.pickFromRack);
                int start = Stage1Subsystem.BlindfoldReset(); //Ethan Slide pt.1
                Stage1Subsystem.setExtPosBlind(500, .8, start); //Ethan Slide pt.2
                Stage1Subsystem.setClawTwistPos(.625);
                Stage1Subsystem.up();
                setState(1003);
                break;
            case 1003:
                if (timer.getElapsedTime() > 6000) {
                    setState(1004);
                }
                break;
            case 1004:
                //FINAL ARM ADJUSTMENT FUNCTION
                Stage2Subsystem.setStage2(Stage2Subsystem.readyClipVal);
                setState(1005);
                break;
            case 1005:
                //2 ARMS MEET FUNCTION
                if (timer.getElapsedTime() < 1800) {}
                else if (timer.getElapsedTime() < 2200) {
                    //Stage1Subsystem.setPos(150); //210 Worked Sometimes
                    Stage1Subsystem.closeMid();
                }
                else {
                    setState(0);
                }
                break;
            case 1006:
                //CLIPPING FUNCTION
                if (timer.getElapsedTime() < 1000) {Stage1Subsystem.setPos(150);}
                else {
                    Stage2Subsystem.setStage2(Stage2Subsystem.clipVal);
                Stage1Subsystem.closeTight();
                setState(1007);}
                break;
            case 1007:
                //RELEASE FUNCTION
                Stage1Subsystem.setPos(460);//Added in comp
                if (timer.getElapsedTime() > 500) Stage1Subsystem.setClawWristPos(0);
                if (timer.getElapsedTime() > 3000) {
                    Stage1Subsystem.open();
                    Stage1Subsystem.up();
                    start = Stage1Subsystem.BlindfoldReset();
                    Stage1Subsystem.setExtPosBlind(600, .8, start);
                    setState(0);
                }
                break;
            case 1008:
                if (timer.getElapsedTime() < 1000) {Stage2Subsystem.readyScore();}
                else if (timer.getElapsedTime() < 2000){
                    Stage2Subsystem.score();
                } else {
                    Stage2Subsystem.score2();
                    setState(0);
                }
                break;


        }
    }

}
