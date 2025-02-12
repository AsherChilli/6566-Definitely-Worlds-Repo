package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Stage1.ArmSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Disabled
@TeleOp(name="Drive", group = "Drive")
public class Drive extends LinearOpMode {

    //Class def


    double gp2Deflator;
    double gp1Deflator;

    int angleTarget = 10;
    int extendTarget = 10;
    double clawZ = 1;
    double clawX = 0.5;




    boolean driveCentric;
    Follower follower;






    @Override
    public void runOpMode() throws InterruptedException {


        //During Initialization:

        follower = new Follower(hardwareMap);

        hardwareMap.get(DcMotorEx.class, leftFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, rightFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, leftRearMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, rightRearMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");



        //commandScheduler = CommandScheduler.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //hMap, name of servo used for claw
        ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap, "clawAngle", "clawDriver", "clawWrist");
        //hMap, name of motor used to change the EXTENSION HEIGHT of the arm/slides
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight");
//        armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem, 0,0 );



        waitForStart();



       follower.startTeleopDrive();



        while (opModeIsActive()) {
            //-----------------------------
            //Input
            // ----------------------------

            gp2Deflator = gamepad2.left_bumper ? 0.5 : 1;
            gp1Deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;

            if (gamepad1.a) {
                driveCentric = false;
            } else if (gamepad1.b) {
                driveCentric = true;
            }
            //Testing clawSubsystem
            if (gamepad2.b) {
                ClawSubsystem.close();
            } else if (gamepad2.a) {
                ClawSubsystem.open();
            }
            if (gamepad2.x) {
                clawZ = 1;
            } else if (gamepad2.y) {
                clawZ = 0;
            }




            clawZ += (Math.pow(gamepad2.left_trigger - gamepad2.right_trigger,3) * 0.025 * gp2Deflator);
            clawX = gamepad2.dpad_right ? 0.5 : gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? 0 : clawX;

            angleTarget += (int) (Math.pow(gamepad2.left_stick_y, 3) * -48 * gp2Deflator * (1- (double)extendTarget / 4000));
            extendTarget += (int) (Math.pow(gamepad2.right_stick_y, 3) * -120 * gp2Deflator);

            /////////////////
            // Automations //
            /////////////////

//            if (gamepad2.dpad_left) highbasket();
//              else //Prepare to score specimen
//                if (gamepad2.dpad_down) picking();
//                else if (gamepad2.left_stick_button) highchamber();




            // ----------------------------
            // Telemetry
            // ----------------------------

            telemetry.addData("Current Angle in Ticks: ", ArmSubsystem.getAnglePos());
            telemetry.addData("Current Angle Target in Ticks: ", angleTarget);


            telemetry.addData("Current Extension in Ticks: ", ArmSubsystem.getExtenderPos());
            telemetry.addData("Current Extension Target in Ticks: ", extendTarget);



            telemetry.addData("Arm Angle: ", ArmSubsystem.getAnglePosDEG());
            telemetry.addData("Arm extension: ", ArmSubsystem.getExtenderPosIN());

            telemetry.addData("Arm subsystem Angle Target:", ArmSubsystem.getAngleTarget());
            telemetry.addData("Arm subsystem Extension Target:", ArmSubsystem.getExtTarget());

            telemetry.addData("X: ", ArmSubsystem.getX());
            telemetry.addData("Y: ", ArmSubsystem.getY());

            telemetry.addData("Claw Wrist: ", ClawSubsystem.angleOfClaw.getPosition());


            telemetry.addLine("Don't Crash!");
            telemetry.addData("Driver Centric?", driveCentric);


            // ----------------------------
            // Updaters
            // ----------------------------

            update();





            follower.update();
            telemetry.addLine("Lock In 🔥 🔥 🔥");
            telemetry.addLine("Improvement Is The First Step to Success");
            telemetry.update();
        }




    }


    private void update() {
        ArmSubsystem.update(angleTarget,extendTarget);
        clawZ = Math.max(ArmSubsystem.getExtenderPos() < 30 ? 0.4: 0, Math.min(1, clawZ));
        clawX = Math.max(0, Math.min(1, clawX));

        ClawSubsystem.setAnglePosition(clawZ);
        ClawSubsystem.setWristPosition(clawX);
        follower.updatePose();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        angleTarget = ArmSubsystem.getAngleTarget();
        extendTarget = ArmSubsystem.getExtTarget();
    }
    private void highbasket() {
        ArmSubsystem.setPos(46,90);
        ClawSubsystem.setAnglePosition(0.5);
        ClawSubsystem.setWristPosition(0.5);
        angleTarget = ArmSubsystem.getAngleTarget();
        extendTarget = ArmSubsystem.getExtTarget();
        ClawSubsystem.open();
        ClawSubsystem.close();
        ArmSubsystem.setPos(0, 0);
        ClawSubsystem.setAnglePosition(0);
        ClawSubsystem.setWristPosition(0);
    }
    private void highchamber() {
        ArmSubsystem.setPos(0,45);
        angleTarget = ArmSubsystem.getAngleTarget();
        extendTarget = ArmSubsystem.getExtTarget();
    }

    private void picking() {
        ArmSubsystem.setPos(22,0);
        clawZ = 0;
        ClawSubsystem.setWristPosition(0.5);
        ClawSubsystem.open();
        angleTarget = ArmSubsystem.getAngleTarget();
        extendTarget = ArmSubsystem.getExtTarget();
    }
}
