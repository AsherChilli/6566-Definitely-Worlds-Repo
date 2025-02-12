package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Stage1.ArmSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Disabled
@Autonomous
public class pathingTestChamber extends OpMode {


    Follower follower;
    private Timer opmodeTimer, pathTimer, armTimer;
    private int pathState;
    private int armState;
    ArmSubsystem armSubsystem;
    ClawSubsystem clawSubsystem;
    CommandScheduler commandScheduler;
    Path tempPath;

    PathChain startToBasket, pickupSample1, returnToBasket1, pickupSample2, returnToBasket2, pickupSample3, returnToBasket3, park;
    autonPosesPedro autonPoses = new autonPosesPedro();

    public void buildPaths() {

        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.basketScore), /* CONTROL POINT ->>> */ new Point(autonPoses.basketParkCP1), new Point(autonPoses.basketPark))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.basketPark.getHeading())
                .build();

        startToBasket = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.startPoseBasket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.startPoseBasket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup1Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup1Basket.getHeading())
                .build();

        returnToBasket1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup1Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup1Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup2Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup2Basket.getHeading())
                .build();

        returnToBasket2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup2Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup2Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup3Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup3Basket.getHeading())
                .build();

        returnToBasket3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup3Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup3Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

    }


    public void autonomousArmUpdate() {
        switch(armState) {
            // Cases for picking up samples
            case -1:
                ClawSubsystem.setWristPosition(0);
                ClawSubsystem.open();
                ArmSubsystem.setPos(18, 18);
                if (!ArmSubsystem.isBusy()) {
                    setArmState(-2);
                }
                break;
            case -2:
                ArmSubsystem.setPos(16,8);
                if (!ArmSubsystem.isBusy() && armTimer.getElapsedTime() > 200) {
                    setArmState(-3);
                }
                break;
            case -3:
                ClawSubsystem.close();
                if (armTimer.getElapsedTime() > 200 && !ArmSubsystem.isBusy()) {
                    setArmState(-4);

                }
                break;
            case -4:
                ArmSubsystem.setPos(10,60);
                if (!ArmSubsystem.isBusy()) {
                    setArmState(0);
                }
                break;
            //cases for scoring samples
            case 1:
                ClawSubsystem.setWristPosition(0);
                ArmSubsystem.setPos(50,100);
                if (!ArmSubsystem.isBusy()) {
                    setArmState(2);
                }
                break;
            case 2:
                ClawSubsystem.setWristPosition(0);
                ArmSubsystem.setPos(50,110);
                if (!ArmSubsystem.isBusy()) {
                    setArmState(3);
                }
                break;
            case 3:
                ClawSubsystem.setWristPosition(1);
                //ClawSubsystem.open();
                if (!ArmSubsystem.isBusy() && armTimer.getElapsedTime() > 400) {
                    setArmState(4);
                }
                break;
            case 4:
                setArmState(0);
                ClawSubsystem.open();
                ClawSubsystem.setWristPosition(0);
                ArmSubsystem.setPos(10, 60);
                break;

        }
    }
    public void autonomousPathUpdate(){

        switch(pathState){

            case 0:
                follower.followPath(startToBasket, true);
                setPathState(1);
                break;
            case 1:
                //Getting to scoring pos
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the basketScore position */
                if(!follower.isBusy()) {
                    ClawSubsystem.setWristPosition(0.5);
                    setPathState(-1);
                    setArmState(1);
                }
                break;
            case -1:
                //placing first sample
                if (armState == 0) {
                    pickupSample1 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.samplePickup1Basket))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.samplePickup1Basket.getHeading())
                            .build();
                    follower.followPath(pickupSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                //moving to pick up first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup1Basket heading */
                if(!follower.isBusy()) {
                    setPathState(-2);
                    setArmState(-1);
                }
                break;
            case -2:
                //picking up first sample
                if (armState == 0) {
                    returnToBasket1 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.basketScore))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.basketScore.getHeading())
                            .build();
                    follower.followPath(returnToBasket1, true);
                    setPathState(3);
                }
                break;
            case 3:
                //scoring first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(!follower.isBusy()) {
                    setPathState(-3);
                    setArmState(1);
                }
                break;
            case -3:
                if (armState == 0) {
                    pickupSample2 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.samplePickup2Basket))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.samplePickup2Basket.getHeading())
                            .build();
                    follower.followPath(pickupSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                //picking up second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup2Basket heading */
                if(!follower.isBusy()) {
                    setPathState(-4);
                    setArmState(-1);
                }
                break;
            case -4:
                if (armState == 0) {
                    returnToBasket2 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.basketScore))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.basketScore.getHeading())
                            .build();
                    follower.followPath(returnToBasket2, true);
                    setPathState(5);
                }
                break;
            case 5:
                //scoring second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(!follower.isBusy()) {
                    setPathState(-5);
                    setArmState(1);
                }
                break;
            case -5:
                if (armState == 0) {
                    pickupSample3 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.samplePickup3Basket))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.samplePickup3Basket.getHeading())
                            .build();
                    follower.followPath(pickupSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                //picking up third sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup3Basket heading */
                if(!follower.isBusy()) {
                    setPathState(-6);
                    setArmState(-1);
                }
                break;
            case -6:
                if (armState == 0) {
                    returnToBasket3 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(autonPoses.basketScore))))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autonPoses.basketScore.getHeading())
                            .build();
                    follower.followPath(returnToBasket3, true);
                    setPathState(7);
                }
                break;
            case 7:
                // scoring third sample
                if(!follower.isBusy()) {
                    setPathState(-7);
                    setArmState(1);
                }
                break;
            case -7:
                if (armState == 0) {
                    //follower.followPath(park, true);
                }
                break;
        }

    }

    public void setPathState(int state){
        armState = 0;
        pathState = state;
        pathTimer.resetTimer();
    }

    public void setArmState(int state) {
        armState = state;
        armTimer.resetTimer();
    }




    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        armTimer = new Timer();

        armSubsystem = new ArmSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);
        commandScheduler = CommandScheduler.getInstance();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(autonPoses.startPoseBasket);
        follower.getPose().setHeading(autonPoses.startPoseBasket.getHeading());
        ClawSubsystem.setAnglePosition(1);
        ArmSubsystem.setPos(2,30);


        buildPaths();
    }

    @Override
    public void init_loop(){
        //Looped
        ClawSubsystem.close();
        ArmSubsystem.update();
        commandScheduler.run();

    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousArmUpdate();
        telemetry.addData("isBusy?", ArmSubsystem.isBusy());
        ArmSubsystem.update();
        telemetry.addData("isBusy?2", ArmSubsystem.isBusy());

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("arm State", armState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }


    @Override
    public void start() {
        //Called once at the start of run, to setup basic stuff
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    public boolean toleranceCheck(double toCheck, double tolerance, double min, double max){

        return toCheck - tolerance > min && toCheck + tolerance < max;


    }


}
