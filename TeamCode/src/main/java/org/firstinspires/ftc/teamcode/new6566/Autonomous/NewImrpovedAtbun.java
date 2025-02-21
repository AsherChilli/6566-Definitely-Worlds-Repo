package org.firstinspires.ftc.teamcode.new6566.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
import org.firstinspires.ftc.teamcode.Stage2.Stage2Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous

public class NewImrpovedAtbun extends OpMode {

    Poses poses = new Poses();

    Follower follower;

    Stage1Subsystem stage1;
    Stage2Subsystem stage2;

    PathChain preload, prepareClip, collectClip, pickupSample, moveToScore;

    //MultipleTelemetry telemetry;

    private int pathState = 0;
    private int armState = 0;
    private int autonState = 0;


    private Timer armTimer = new Timer();
    private Timer autonTimer = new Timer();
    private Timer pathTimer = new Timer();

    //Class of sampleDetection


    //For 1 time init thingy
    @Override
    public void init() {
        //This justs make sure that we can see the telemetry on both dash and DS
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Sets up epic driver thingy
        follower = new Follower(hardwareMap);
        follower.setStartingPose(poses.start);
        buildPaths();

        stage2 = new Stage2Subsystem(hardwareMap);
        stage1 = new Stage1Subsystem(hardwareMap);

        Stage2Subsystem.setAngTarget(1200);
        Stage2Subsystem.setAngPower(0.7);
        setAutonState(1);
        Stage2Subsystem.lowerCams();

        //Vision Portal INIT

    }

    //For multiple loopy things
    @Override
    public void init_loop(){

        //Determines whether we are RED or BLUE
        if(gamepad1.dpad_up){Stage1Subsystem.setRed();}
        else if(gamepad1.dpad_down){Stage1Subsystem.setBlue();}


        Stage1Subsystem.update();
        Stage2Subsystem.update();

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
        Stage1Subsystem.update();
        Stage2Subsystem.update();
        Stage2Subsystem.stage2Updater();
        follower.update();
        follower.updatePose();
        autonomousPathUpdate();
        autonomousUpdate();


        telemetry.addData("Path State: ", pathState);
        telemetry.update();

    }

    void setPathState(int num){pathState = num;}

    void autonomousPathUpdate(){

        switch (pathState){
            case 0 :
                break;

            //Make robot go forward and
            case(1):
                /*
                We have to:
                Move to score the preload
                Raise the arm to score
                Some kind of score function
                 */
                //Stage2Subsystem.readyScore();
                follower.followPath(preload, true);
                setPathState(2);
                break;

            case(2):
                if (!follower.isBusy())
                    setPathState(0);
                break;
            case(3):
                /*
                We have to:
                Flip the arm to it's top position
                Move to the preloadClip spot
                Ensure cams are lowered (servos are powered to the low end)
                 */
                if (pathTimer.getElapsedTime() < 1000) Stage2Subsystem.score();
                else follower.followPath(prepareClip);
                setPathState(4);
                break;
            case(4):
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2000)
                    setPathState(0);
                break;
            case(5):
                /*
                We have to:
                Move into the wall
                Raise the cams (both of them please, all the way)
                 */
                Stage1Subsystem.pickupSample();
                follower.followPath(collectClip);
                setPathState(6);
                break;
            case(6):
                if (!follower.isBusy())
                    if (pathTimer.getElapsedTime() < 1000) {
                        Stage2Subsystem.raiseCams();
                    } else {
                        setPathState(0);
                    }
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
                if (pathTimer.getElapsedTime() < 500) {}
                else follower.followPath(pickupSample);
                setPathState(8);
                break;
            case(8):
                if (!follower.isBusy())
                    setPathState(0);
                break;
            case(9):
                /*
                We have to:
                Clip the sample
                Move to scoring position
                Get arm into scoring position, while moving (efficiency!)
                Score function epicness
                 */
                setPathState(0);
                break;
            case(-1):
                /*
                We have to:
                Make the thing cycle.
                 */

        }

    }

    void setArmState(int num) {
        armState = num;
        armTimer.resetTimer();
    }

    void autonomousArmUpdate() {
        switch (armState) {
            case 0:
                break;
            case 1000:
                Stage2Subsystem.setStage2(Stage2Subsystem.readyPickFromRack);
                Stage1Subsystem.up();
                Stage1Subsystem.close();
                setArmState(1001);
                break;
            case 1001:
                if (armTimer.getElapsedTime() > 1800) {
                    setArmState(1002);
                }
                break;
            case 1002:
                Stage2Subsystem.setStage2(Stage2Subsystem.pickFromRack);
                int start = Stage1Subsystem.BlindfoldReset(); //Ethan Slide pt.1
                Stage1Subsystem.setExtPosBlind(500, .8, start); //Ethan Slide pt.2
                Stage1Subsystem.setClawTwistPos(.625);
                Stage1Subsystem.up();
                setArmState(1003);
                break;
            case 1003:
                if (armTimer.getElapsedTime() > 6000) {
                    setArmState(1004);
                }
                break;
            case 1004:
                //FINAL ARM ADJUSTMENT FUNCTION
                Stage2Subsystem.setStage2(Stage2Subsystem.readyClipVal);
                setArmState(1005);
                break;
            case 1005:
                //2 ARMS MEET FUNCTION
                if (armTimer.getElapsedTime() < 1800) {
                } else if (armTimer.getElapsedTime() < 2200) {
                    //Stage1Subsystem.setPos(150); //210 Worked Sometimes
                    Stage1Subsystem.closeMid();
                } else {
                    setArmState(1006);
                }
                break;
            case 1006:
                //CLIPPING FUNCTION
                if (armTimer.getElapsedTime() < 1000) {
                    Stage1Subsystem.setPos(150);
                } else {
                    Stage2Subsystem.setStage2(Stage2Subsystem.clipVal);
                    Stage1Subsystem.closeTight();
                    setArmState(1007);
                }
                break;
            case 1007:
                //RELEASE FUNCTION
                if (armTimer.getElapsedTime() > 500) Stage1Subsystem.setClawWristPos(0);
                if (armTimer.getElapsedTime() > 3000) {
                    Stage1Subsystem.open();
                    Stage1Subsystem.up();
                    start = Stage1Subsystem.BlindfoldReset();
                    Stage1Subsystem.setExtPosBlind(600, .8, start);
                    setArmState(0);
                }
                break;
            case 1008:
                if (armTimer.getElapsedTime() < 1000) {Stage2Subsystem.readyScore();}
                else if (armTimer.getElapsedTime() < 2000){
                    Stage2Subsystem.score();
                } else {
                    Stage2Subsystem.score2();
                    setArmState(0);
                }
                break;


        }
    }


    void setAutonState(int num) {
        autonState = num;
        autonTimer.resetTimer();
    }
    void autonomousUpdate() {
        switch (autonState) {
            case 0:
                break;
            case 1:
                setPathState(1);
                //Stage2Subsystem.readyScore();
                setAutonState(2);


                break;
            case 2:
                if (pathState == 0) {
                    setAutonState(3);
                }
                break;
            case 3:
                //setArmState(1008);

                setAutonState(4);
                break;
            case 4:
                if (armState == 0)
                setAutonState(5);
                break;
            case 5:
                setPathState(3);
                setAutonState(6);
                break;
            case 6:
                if (pathState == 0) {
                    setAutonState(7);
                }
                break;
            case 7:
                setPathState(5);
                setAutonState(8);
                break;
            case 8:
                if (pathState == 0) {
                    setAutonState(9);
                }
                break;
            case(9) :
                setPathState(7);
                setAutonState(10);
            case(10): break;

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
