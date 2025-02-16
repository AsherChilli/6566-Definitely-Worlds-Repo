package org.firstinspires.ftc.teamcode.new6566.Autonomous;

import android.media.audiofx.DynamicsProcessing;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.Processors.sampleProcessor;
import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous

public class NewImrpovedAtbun extends OpMode {

    Poses poses = new Poses();

    Follower follower;
    VisionPortal visionPortal;

    Stage1Subsystem stage1 = new Stage1Subsystem(hardwareMap);
    Stage2Subsystem stage2 = new Stage2Subsystem(hardwareMap);

    PathChain preload, prepareClip, collectClip, pickupSample, moveToScore;

    MultipleTelemetry telemetry;

    int pathState = 0;

    private int stage2State = 0;

    private final int pickFromRack = 1000;

    private Timer stage2timer = new Timer();
    private Timer pathTimer = new Timer();

    //Class of sampleDetection
    sampleProcessor processor = new sampleProcessor();

    //Some variable for color
    sampleProcessor.Color color = sampleProcessor.Color.RED;

    //For 1 time init thingy
    @Override
    public void init() {
        //This justs make sure that we can see the telemetry on both dash and DS
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Sets up epic driver thingy
        follower = new Follower(hardwareMap);
        buildPaths();

        //Vision Portal INIT
        visionPortal = new VisionPortal.Builder()
                //Get camera from hMap
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //Add all needed processors (hand written)
                .addProcessor(processor)
                //Set camera resolution
                .setCameraResolution(new Size(640, 480))
                //No idea
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //Enable streaming to Dhub and FTC Dashboard (?)
                .enableLiveView(true)
                //If all proccesors disabled, stop streaming
                .setAutoStopLiveView(true)
                //finish
                .build();
    }

    //For multiple loopy things
    @Override
    public void init_loop(){

        //Determines whether we are RED or BLUE
        if(gamepad1.dpad_up){color = sampleProcessor.Color.RED;}
        else if(gamepad1.dpad_down){color = sampleProcessor.Color.BLUE;}

        //Sets the processor to be for that color
        sampleProcessor.setColor(color);

        //Telemetry
        telemetry.addLine("For changing between RED and BLUE:");
        telemetry.addLine("Use DPAD_UP for RED");
        telemetry.addLine("Use DPAD_DOWN for BLUE");
        telemetry.addData("Current Color: ", color);
        telemetry.update();
    }

    @Override
    public void loop() {
        //I loveee my autonomous path update
        autonomousPathUpdate();


        telemetry.addData("Path State: ", pathState);
        telemetry.update();

    }

    void setState(int num){pathState = num;}

    void autonomousPathUpdate(){

        switch (pathState){

            //Make robot go forward and
            case(1):
                /*
                We have to:
                Move to score the preload
                Raise the arm to score
                Some kind of score function
                 */
                setState(2);
                break;

            case(2):
                /*
                We have to:
                Flip the arm to it's top position
                Move to the preloadClip spot
                Ensure cams are lowered (servos are powered to the low end)
                 */
                setState(3);
                break;
            case(3):
                /*
                We have to:
                Move into the wall
                Raise the cams (both of them please, all the way)
                 */
                setState(4);
                break;
            case(4):
                /*
                We have to:
                Move to pickup the sample
                Grab a clip in the clipClaw
                Lower sampleClaw
                Extend out slides
                Close sampleClaw
                Retract back in
                Have claw(s) in their clipping position
                 */
                setState(5);
                break;
            case(5):
                /*
                We have to:
                Clip the sample
                Move to scoring position
                Get arm into scoring position, while moving (efficiency!)
                Score function epicness
                 */
                setState(-1);
                break;
            case(-1):
                /*
                We have to:
                Make the thing cycle.
                 */

        }

    }

    private void setStage2(int status) {
        stage2State = status;
        stage2timer.resetTimer();
    }
    private void stage2Updater() {
        switch (stage2State) {
            case 0:
                break;
            case 1:
                break;
            case pickFromRack:
                Stage2Subsystem.setAngTarget(-670 + 1493);
                Stage2Subsystem.setAngPower(0.7);
                Stage2Subsystem.setClawPos(0);
                if (Stage2Subsystem.getAngPos() == -670 + 1493 && stage2timer.getElapsedTime() > 1500) {
                    Stage2Subsystem.setClawPos(0.5);
                    Stage2Subsystem.setAngTarget(-600 + 1493);
                } else if (stage2timer.getElapsedTime() > 2500) {
                    setStage2(1001);
                }
                break;


            case 1001:
                Stage2Subsystem.holdOpenMax();
                Stage2Subsystem.setClawWristPos(0.65);


                break;
            case 1002:
                Stage2Subsystem.setClawWristPos(0.65);
                Stage2Subsystem.holdOpenMax();
                Stage2Subsystem.setAngTarget(0 + 1493);
                Stage2Subsystem.setAngPower(0.3);
                break;

            case 1003:
                Stage2Subsystem.setClawPos(.46);
                Stage2Subsystem.setClawWristPos(.5);
                if (stage2timer.getElapsedTime() > 250) {
                    Stage2Subsystem.holdClose();
                }
                break;
            case 1004:
                Stage2Subsystem.setClawWristPos(.6);
                Stage2Subsystem.holdCloseTight();
                if (stage2timer.getElapsedTime() > 250) {
                    Stage2Subsystem.setAngTarget(-300 + 1493);
                    Stage2Subsystem.setAngPower(.4);
                }
                break;
            case 1005:
                Stage2Subsystem.setClawWristPos(.825);
                break;
            case 1006:
                Stage2Subsystem.setAngTarget(55 + 1493);
                break;
        }
    }



        void buildPaths(){
        //Sets up preload path
        preload = follower.pathBuilder()
                .addPath((new BezierLine(new Point(poses.start), new Point(poses.score))))
                .setConstantHeadingInterpolation(0)
                .build();
        //Gets ready to prepareClip
        prepareClip = follower.pathBuilder()
                .addPath((new BezierLine(new Point(poses.score), new Point(poses.almostPickupClip))))
                .setConstantHeadingInterpolation(0)
                .build();
        //Collects the clips
        collectClip = follower.pathBuilder()
                .addPath((new BezierLine(new Point(poses.almostPickupClip), new Point(poses.pickupClip))))
                .setConstantHeadingInterpolation(0)
                .build();
        //Goes to pickup the sample
        pickupSample = follower.pathBuilder()
                .addPath((new BezierLine(new Point(poses.pickupClip), new Point(poses.pickupSample))))
                .setConstantHeadingInterpolation(0)
                .build();
        //Move bot to score
        moveToScore = follower.pathBuilder()
                .addPath((new BezierLine(new Point(poses.pickupSample), new Point(poses.score))))
                .setConstantHeadingInterpolation(0)
                .build();
    }
}
