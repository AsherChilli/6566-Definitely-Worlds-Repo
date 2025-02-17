package org.firstinspires.ftc.teamcode.new6566;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.new6566.SleekClippaHardware;

@TeleOp(name="SleekClippaDrive", group = "TeleOp")
public class SleekClippaDrive extends OpMode {
    SleekClippaHardware r = new SleekClippaHardware();
    private ElapsedTime runtime = new ElapsedTime();

    //Stage1Subsystem stage1 = new Stage1Subsystem(hardwareMap);
    //Stage2Subsystem stage2 = new Stage2Subsystem(hardwareMap);

    //Clip Vars
    double holdOpenMax = .475;//.45
    double holdClose = .35;//.3
    double holdCloseTight = .3;//.2
    boolean clipping = false;

    @Override
    public void init() {
        r.init_robot(this);
   }

   @Override
   public void init_loop() {
        telemetry.addData("arm extension", r.ExtendLeft.getCurrentPosition());
        telemetry.update();
   }

    @Override
    public void loop() {

        /*
         *
         *Gamepad 1
         *
         */

        // Keep existing drive code unchanged
        double deflator = 0.7;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;
        double angle = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double leftFront = power * cos / max + rotation;
        double rightFront = power * sin / max - rotation;
        double leftRear = power * sin / max + rotation;
        double rightRear = power * cos / max - rotation;

        if (power + Math.abs(rotation) > 1) {
            double scale = power + Math.abs(rotation);
            leftFront /= scale;
            rightFront /= scale;
            leftRear /= scale;
            rightRear /= scale;
        }

        r.TopLeft.setPower(leftFront * deflator);
        r.TopRight.setPower(rightFront * deflator);
        r.BottomLeft.setPower(leftRear * deflator);
        r.BottomRight.setPower(rightRear * deflator);

        //Extend or retract arm
        if (gamepad1.right_trigger > 0 && r.ExtendRight.getCurrentPosition() < 3000) {
            r.ExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.ExtendLeft.setPower(-1 * gamepad1.right_trigger);
            r.ExtendRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.ExtendRight.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0 && r.ExtendRight.getCurrentPosition() > 0) {
            r.ExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.ExtendLeft.setPower(gamepad1.left_trigger);
            r.ExtendRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.ExtendRight.setPower(-1 * gamepad1.left_trigger);
        }
        else if (!clipping) {
            //r.ExtendRight.setTargetPosition(r.ExtendRight.getCurrentPosition());
            //r.ExtendLeft.setTargetPosition(r.ExtendLeft.getCurrentPosition());
            r.ExtendRight.setPower(0);
            r.ExtendLeft.setPower(0);

        }

        //Close or Open Claw
        if (gamepad1.right_bumper) {
            //Close
            r.GameClaw.setPosition(.46);//.525
        }
        else if (gamepad1.left_bumper) {
            r.GameClaw.setPosition(.8);
        }

        //Wrist motions
        if (gamepad1.dpad_up) {
            r.GameWrist.setPosition(.45 + .25);//.4before
        }
        else if (gamepad1.dpad_down) {
            r.GameWrist.setPosition(.75 + .25);//.8
        }

        //Twist motions
        if (gamepad1.dpad_left && r.GameTwist.getPosition() <= .99) {
            r.GameTwist.setPosition(r.GameTwist.getPosition() + .01);
        }
        else if (gamepad1.dpad_right && r.GameTwist.getPosition() >= 0.01) {
            r.GameTwist.setPosition(r.GameTwist.getPosition() - .01);
        }
        else if (gamepad1.b) {
            //Reset to straight
            r.GameTwist.setPosition(.625);
        }

        /*
         *
         *Gamepad2 Stuff
         *
         */


        //Move Arm to Clip Rack
        if (gamepad2.dpad_right){
            r.ClipWrist.setPosition(.65);
            r.ClipHold.setPosition(holdOpenMax);
            r.ClipArm.setTargetPosition(0 + 1493);
            r.ClipArm.setPower(.3);
            r.ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Take Clip Claw off rack
        if (gamepad2.dpad_left) {
            r.ClipWrist.setPosition(.6);
            r.ClipHold.setPosition(holdCloseTight);
            double time = runtime.time();
            while (runtime.time() - time <= .25) {
                continue;
            } if (runtime.time() - time > .25) {
                r.ClipArm.setTargetPosition(-300 + 1493);
                r.ClipArm.setPower(.4);
            }
        }

        //Ensure claw will not break :pray:
        if(r.ExtendRight.getCurrentPosition() < 1150 || r.ExtendLeft.getCurrentPosition() < 1150){

            r.GameWrist.setPosition(.7);

        }


        //Score Position
        if (gamepad2.dpad_up) {
            r.ClipArm.setTargetPosition(-670 + 1493);
            r.ClipArm.setPower(.7);
            r.ClipHold.setPosition(0);
            r.ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           if(r.ClipArm.getCurrentPosition() == -670 + 1493) {
               r.ClipHold.setPosition(0.5);
               r.ClipArm.setTargetPosition(-600 + 1493);
               r.ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }
        }

        //Turn Wrist to pass rack position
        if (gamepad2.x) {
            r.ClipWrist.setPosition(.825);//.8
        }

        //CLip Arm to Clip Position
        if (gamepad2.dpad_down) {
            r.ClipArm.setTargetPosition(55 + 1493);
        }

        //Grab or release clip
        if (gamepad2.right_bumper) {
            //Grab
            r.ClipWrist.setPosition(.5);
            double time = runtime.time();
            while (runtime.time() - time <= .25){
                continue;
            } if (runtime.time() - time > .25) {
                r.ClipHold.setPosition(holdClose);
            }
        }
        else if (gamepad2.left_bumper) {
            //Release
            r.ClipHold.setPosition(holdOpenMax);
            r.ClipWrist.setPosition(.65);
        }

        //Collect Clips
        if (gamepad2.right_trigger > .5){
            r.ClipCamLeft.setPosition(0);
            r.ClipCamRight.setPosition(1);
        }
        else if (gamepad2.left_trigger > .5){
            r.ClipCamLeft.setPosition(0.7);
            r.ClipCamRight.setPosition(0.4);
        }

        //CLIP FUNCTION
        if (gamepad2.y) {
            clipping = true;
            r.GameClaw.setPosition(.3);//.4
            r.ClipHold.setPosition(holdCloseTight);

            double time = runtime.time();
            while (runtime.time() - time <= .5) {
                continue;
            }
            if (runtime.time() - time > .5) {
                r.ClipWrist.setPosition(1);
            }
            while (runtime.time() - time <= 1){
                continue;
            }
            if (runtime.time() - time > 1) {
                r.GameWrist.setPosition(0);//.4?
            }
        }

        //RELEASE FUNCTION
        if (gamepad2.a) {
            clipping = false;
            r.GameClaw.setPosition(.6);
            r.GameWrist.setPosition(.7);
            r.ClipArm.setTargetPosition(100 + 1493);
            r.ClipArm.setPower(.6);
            r.ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.a){
            r.ClipWrist.setPosition(0.15);
            r.ClipArm.setTargetPosition(-895 + 1493);
        }

        telemetry.addLine("Arm Position: " + String.valueOf(r.ClipArm.getCurrentPosition()) );
        telemetry.addLine("Extendo Position: " + String.valueOf(r.ExtendRight.getCurrentPosition()) );
        telemetry.addLine("Twist Position: " + String.valueOf(r.GameTwist.getPosition()) );

    }
}
