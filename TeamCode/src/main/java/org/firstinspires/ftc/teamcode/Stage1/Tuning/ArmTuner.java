package org.firstinspires.ftc.teamcode.Stage1.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;
@Config
@TeleOp(name = "ArmTunerStage1", group = "Tuning")
public class ArmTuner extends LinearOpMode {




    public static double x = 1;
    public static double y = 0;
    public static double pAngle = 0.005, iAngle = 0.0, dAngle = 0.0008, fAngle = 0.1, pExtend = 0.015, iExtend = 0, dExtend = 0.0004;



    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();


        Stage1Subsystem stage1Subsystem = new Stage1Subsystem(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight");



        while(!isStopRequested()){



            if (pAngle != Stage1Subsystem.getAngleP() || iAngle != Stage1Subsystem.getAngleI() || dAngle != Stage1Subsystem.getAngleD() || fAngle != Stage1Subsystem.getAngleFeedForward() || pExtend != Stage1Subsystem.getExtendP() || iExtend != Stage1Subsystem.getExtendI() || dExtend != Stage1Subsystem.getExtendD()) {
                Stage1Subsystem.setAnglePID(pAngle, iAngle, dAngle);
                Stage1Subsystem.setExtendPID(pExtend, iExtend, dExtend);
            }

            Stage1Subsystem.setPos(new Vector2d(x,y));
            telemetry.addData("isBusy?: ", Stage1Subsystem.isBusy());
            Stage1Subsystem.update();


            telemetry.addData("Angle Ticks: ", Stage1Subsystem.getAnglePos());
            telemetry.addData("Angle Degrees: ", Stage1Subsystem.getAnglePosDEG());
            telemetry.addData("Extension Ticks: ", Stage1Subsystem.getExtenderPos());
            telemetry.addData("Extension Inches: ", Stage1Subsystem.getExtenderPosIN());

            telemetry.addData("Current REAL Angle Target", Stage1Subsystem.getAngleTarget());
            telemetry.addData("Current REAL Extend Target", Stage1Subsystem.getExtTarget());

            telemetry.addData("Current Set X: ",x );
            telemetry.addData("Current Set Y: ",y );

            telemetry.addData("isBusy?:2", Stage1Subsystem.isBusy());



            telemetry.update();



        }



    }
}
