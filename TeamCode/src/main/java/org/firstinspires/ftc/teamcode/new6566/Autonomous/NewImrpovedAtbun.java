package org.firstinspires.ftc.teamcode.new6566.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.Processors.sampleProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous

public class NewImrpovedAtbun extends OpMode {

    Poses poses = new Poses();

    Follower follower;
    VisionPortal visionPortal;

    PathChain preload, prepareClip, collectClip, pickupSample, moveToScore;

    MultipleTelemetry telemetry;

    int pathState = 0;

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
