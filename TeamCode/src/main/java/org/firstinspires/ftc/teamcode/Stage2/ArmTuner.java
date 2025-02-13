package org.firstinspires.ftc.teamcode.Stage2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ArmTuner extends LinearOpMode {




    public static double x = 1;
    public static double y = 0;
    public static double pAngle = 0.005, iAngle = 0.0, dAngle = 0.0008, fAngle = 0.1, pExtend = 0.015, iExtend = 0, dExtend = 0.0004;



    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();


        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight");



        while(!isStopRequested()){



            if (pAngle != ArmSubsystem.getAngleP() || iAngle != ArmSubsystem.getAngleI() || dAngle != ArmSubsystem.getAngleD() || fAngle != ArmSubsystem.getAngleFeedForward() || pExtend != ArmSubsystem.getExtendP() || iExtend != ArmSubsystem.getExtendI() || dExtend != ArmSubsystem.getExtendD()) {
                ArmSubsystem.setAnglePID(pAngle, iAngle, dAngle);
                ArmSubsystem.setExtendPID(pExtend, iExtend, dExtend);
            }

            ArmSubsystem.setPos(new Vector2d(x,y));
            telemetry.addData("isBusy?: ", ArmSubsystem.isBusy());
            ArmSubsystem.update();


            telemetry.addData("Angle Ticks: ", ArmSubsystem.getAnglePos());
            telemetry.addData("Angle Degrees: ", ArmSubsystem.getAnglePosDEG());
            telemetry.addData("Extension Ticks: ", ArmSubsystem.getExtenderPos());
            telemetry.addData("Extension Inches: ", ArmSubsystem.getExtenderPosIN());

            telemetry.addData("Current REAL Angle Target", ArmSubsystem.getAngleTarget());
            telemetry.addData("Current REAL Extend Target", ArmSubsystem.getExtTarget());

            telemetry.addData("Current Set X: ",x );
            telemetry.addData("Current Set Y: ",y );

            telemetry.addData("isBusy?:2", ArmSubsystem.isBusy());



            telemetry.update();



        }



    }
}
