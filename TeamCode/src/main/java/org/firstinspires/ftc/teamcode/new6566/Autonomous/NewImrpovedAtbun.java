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


    //For 1 time init thingy
    @Override
    public void init() {
        //This justs make sure that we can see the telemetry on both dash and DS
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Sets up epic driver thingy
        follower = new Follower(hardwareMap);
        buildPaths();

        //Vision Portal INIT

    }

    //For multiple loopy things
    @Override
    public void init_loop(){

        //Determines whether we are RED or BLUE
        if(gamepad1.dpad_up){Stage1Subsystem.setRed();}
        else if(gamepad1.dpad_down){Stage1Subsystem.setBlue();}

        //Sets the processor to be for that color



        //Telemetry
        telemetry.addLine("For changing between RED and BLUE:");
        telemetry.addLine("Use DPAD_UP for RED");
        telemetry.addLine("Use DPAD_DOWN for BLUE");
        telemetry.addData("Current Color: ", Stage1Subsystem.getColor());
        telemetry.update();
    }

    @Override
    public void loop() {
        //I loveee my autonomous path update
        autonomousPathUpdate();
        Stage1Subsystem.update();
        Stage2Subsystem.update();
        Stage2Subsystem.


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
                follower.followPath(preload, true);
                setState(2);
                break;

            case(2):
                if (!follower.isBusy())
                    setState(3);
                break;
            case(3):
                /*
                We have to:
                Flip the arm to it's top position
                Move to the preloadClip spot
                Ensure cams are lowered (servos are powered to the low end)
                 */
                follower.followPath(prepareClip);
                setState(4);
                break;
            case(4):
                if (!follower.isBusy())
                    setState(5);
                break;
            case(5):
                /*
                We have to:
                Move into the wall
                Raise the cams (both of them please, all the way)
                 */
                follower.followPath(collectClip);
                setState(6);
                break;
            case(6):
                if (!follower.isBusy())
                    setState(7);
                break;
            case(7):
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
                follower.followPath(pickupSample);
                setState(8);
                break;
            case(8):
                if (!follower.isBusy())
                    setState(9);
                break;
            case(9):
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
