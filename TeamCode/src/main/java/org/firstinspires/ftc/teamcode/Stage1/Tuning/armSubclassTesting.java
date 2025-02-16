package org.firstinspires.ftc.teamcode.Stage1.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stage1.Stage1Subsystem;

@Config
@TeleOp
@Disabled
public class armSubclassTesting extends LinearOpMode {

    private PIDController controller;


    public static double x = 18;
    public static double y = 0;

    private double[] position;

    private final double ticks_in_degree = (751.8 * 4) / 360;
    private final double ticks_in_inch = 537.7 / (112 / 25.4);


    CommandScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {


        //scheduler = scheduler.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();


        Stage1Subsystem stage1Subsystem = new Stage1Subsystem(hardwareMap);
        //armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem, 0, 0);

        //armSubsystem.setDefaultCommand(armPIDFCommand);


        while(!isStopRequested()){

            //armSubsystem.setArmTarget((int) x);

            //scheduler.run();




            //telemetry.addData("Angle Ticks", stage1Subsystem.getAnglePos());
            //telemetry.addData("Angle Degrees", stage1Subsystem.getAnglePosDEG());
            telemetry.addData("Extension Ticks", stage1Subsystem.getExtenderPos());
            telemetry.addData("Extension Inches", stage1Subsystem.getExtenderPosIN());


            //telemetry.addData("Current REAL Angle Target", stage1Subsystem.getAngleTargetDG());
            telemetry.addData("Current REAL Extend Target", stage1Subsystem.getExtTargetIN());

            telemetry.addData("Current Set X: ",x );
            telemetry.addData("Current Set Y: ",y );

//            position = armSubsystem.getPosition();

            telemetry.addData("Current X: ", position[0]);
            telemetry.addData("Current y: ", position[1]);

            telemetry.addData("Angle Target: ", (int) (Math.toDegrees(Math.atan(y/x)) * ticks_in_degree));
            telemetry.addData("Extension Target: ", (int) ((int) -(Math.sqrt((x)*(x) + y*y)-18) * ticks_in_inch));
            telemetry.update();



        }



    }
}
