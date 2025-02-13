package org.firstinspires.ftc.teamcode.new6566;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="Simple Odometry Auto", group="Robot")
public class NewAutonomous extends LinearOpMode {
    // Drive Motors
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private double gTTracker = .5;
    // Mechanism Motors/Servos
    private DcMotorEx Elbow;
    private DcMotorEx ClipArm;
    private DcMotorEx ChainMotor;
    private DcMotorEx ExtendRight;
    private DcMotorEx ExtendLeft;
    private Servo ClipWrist;
    private Servo ClipHold;
    private Servo GameWrist;
    private Servo GameTwist;
    private Servo GameClaw;
    private Servo Tilter;
    private DcMotorEx ClipElbow;
    private Servo Rack;
    // Sensors
    private DcMotorEx leftPod;    // Left odometry pod
    private DcMotorEx rightPod;   // Right odometry pod
    private IMU imu;

    // Constants
    private static final double COUNTS_PER_INCH = 30.0; // Adjust for your odometry pods
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.6;
    private static final double STRAFE_SPEED = 1;

    private ElapsedTime runtime = new ElapsedTime();

    private int stage2Status = 0;

    private final int pickFromRack = 1000;

    //Clip Vars
    double holdOpenMax = .475;//.45
    double holdClose = .35;//.3
    double holdCloseTight = .3;//.2
    boolean clipping = false;

    private Timer stage2timer = new Timer();


    private void setStage2(int status) {
        stage2Status = status;
        stage2timer.resetTimer();
    }
    private void stage2Updater() {
        switch (stage2Status) {
            case 0:
                break;
            case 1:
                break;
            case pickFromRack:
                ClipWrist.setPosition(.65);
                ClipHold.setPosition(holdOpenMax);
                ClipArm.setTargetPosition(1493);
                ClipArm.setPower(.3);
                ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (stage2timer.getElapsedTime() > 250) {
                    setStage2(1001);
                }
                break;
            case 1001:
                break;
        }
    }


    private void moveToScoringPosition() {
        telemetry.addData("Status", "Starting scoring sequence");
        telemetry.update();

        // 1. Set initial positions
        GameWrist.setPosition(1);
        GameTwist.setPosition(0.5);  // Neutral position
        setGameClawPosition(false);  // Ensure pixel is gripped
        sleep(200);

        // 2. Drive forward FIRST - this was being skipped
        telemetry.addData("Status", "Starting drive");
        telemetry.update();
        driveForward(144, DRIVE_SPEED);
        sleep(300);
        telemetry.addData("Status", "Drive complete");
        telemetry.update();

        // 3. Raise arm to scoring height
        telemetry.addData("Status", "Moving chain");
        telemetry.update();

        // 4. Final adjustments
        updateElbowPosition(28 * 75 * 0.75, true);
        waitForMotorCompletion(Elbow);

        GameWrist.setPosition(0.62);
        sleep(200);

        telemetry.addData("Status", "Scoring position complete");
        telemetry.update();
    }

    private void scorePixel() {
        // Open game claw to release pixel
        setGameClawPosition(true);
        sleep(300);

        // Small backup to clear the board
        strafeDrive(-2, STRAFE_SPEED * 0.3);
        sleep(200);

        // Lower arm slightly
        moveChainToPosition((int)(28 * 100 * 0.5), 0.6);
        waitForMotorCompletion(ChainMotor);
    }

    private void returnToNeutral() {
        // Back away from board
        strafeDrive(-6, STRAFE_SPEED);
        sleep(200);

        // Lower arm to travel position
        moveChainToPosition((int)(28 * 100 * 0.2), 0.8);
        waitForMotorCompletion(ChainMotor);

        // Reset game claw position
        GameWrist.setPosition(1);
        GameTwist.setPosition(0.5);
        setGameClawPosition(false);

        // Return elbow to neutral
        updateElbowPosition(28 * 75 * 0.4, true);
        waitForMotorCompletion(Elbow);
    }


    // Example usage in autonomous mode:
    @Override
    public void runOpMode() {
        initializeHardware();
        setStartingPostion("init");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            resetEncoders();
            resetIMU();

            // Example full scoring sequence
            performNewScoringSequence();

            // Additional scoring sequences can be added here
            // You might want to add pixel collection between scores
        }
    }

    private void initializeHardware() {
        // Initialize drive motors with correct directions for X-pattern
        frontLeft = hardwareMap.get(DcMotorEx.class, "TLM");
        frontRight = hardwareMap.get(DcMotorEx.class, "TRM");
        backLeft = hardwareMap.get(DcMotorEx.class, "BLM");
        backRight = hardwareMap.get(DcMotorEx.class, "BRM");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize mechanism motors/servos
        ExtendLeft = hardwareMap.get(DcMotorEx.class, "EXL");
        ExtendRight = hardwareMap.get(DcMotorEx.class, "EXR");
        Elbow = hardwareMap.get(DcMotorEx.class, "ELB");
        ClipArm = hardwareMap.get(DcMotorEx.class, "CPA");
        ChainMotor = hardwareMap.get(DcMotorEx.class, "CHM");
        ClipWrist = hardwareMap.get(Servo.class, "CWR");
        ClipHold = hardwareMap.get(Servo.class, "CLH");
        GameWrist = hardwareMap.get(Servo.class, "GMW");
        GameTwist = hardwareMap.get(Servo.class, "GMT");
        GameClaw = hardwareMap.get(Servo.class, "GMC");
        Tilter = hardwareMap.get(Servo.class, "TLT");
        Rack = hardwareMap.get(Servo.class,"RCK");
        ClipElbow = hardwareMap.get(DcMotorEx.class,"CEB");
        // Initialize odometry pods
        leftPod = hardwareMap.get(DcMotorEx.class, "TRM");
        rightPod = hardwareMap.get(DcMotorEx.class, "BRM");

        // Initialize IMU

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set motor behaviors
        setMotorDefaults(frontLeft, frontRight, backLeft, backRight);
    }

    private void setMotorDefaults(DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }



    private void strafeDrive(double inches, double speed) {
        int targetCounts = (int)(inches * COUNTS_PER_INCH);

        // For strafing, opposite wheels move in opposite directions
        frontLeft.setTargetPosition(-targetCounts);
        frontRight.setTargetPosition(targetCounts);
        backLeft.setTargetPosition(targetCounts);
        backRight.setTargetPosition(targetCounts);

        runToPosition(speed);
    }

    private void turnToAngle(double targetAngle, double speed) {
        // Constants
        final double TOLERANCE = 2.0;
        final double MIN_POWER = 0.07;
        final double MAX_POWER = speed;

        while (opModeIsActive()) {
            // Get current angle and REVERSE it to match expected direction
            double currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngle - currentAngle;

            // Normalize error to -180 to 180
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Current (Reversed)", currentAngle);
            telemetry.addData("Error", error);

            if (Math.abs(error) <= TOLERANCE) {
                telemetry.addLine("Within tolerance - stopping");
                telemetry.update();
                break;
            }

            // Calculate turn power
            double turnPower;
            if (Math.abs(error) > 45) {
                turnPower = MAX_POWER;
            } else {
                turnPower = MIN_POWER + (MAX_POWER - MIN_POWER) * Math.sqrt(Math.abs(error) / 45.0);
            }
            turnPower *= Math.signum(error);

            telemetry.addData("Turn Power", turnPower);
            telemetry.update();

            // Apply power to motors
            frontLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backLeft.setPower(turnPower);
            backRight.setPower(-turnPower);

            sleep(10);
        }

        stopDrive();
    }
    private void setStartingPostion(String postion){
        // Only set servos and basic positions
        GameWrist.setPosition(1);
        GameTwist.setPosition(gTTracker);
        ClipWrist.setPosition(0);
        ClipHold.setPosition(0.35);  // Closed position from performClippingSequence
        sleep(200);  // Give time for servo to move


        // Initialize lastElbowPosition
        int lastElbowPosition = Elbow.getCurrentPosition();
    }
    private void runToPosition(double speed) {

        // Set mode to RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set drive power
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        // Wait for completion
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Status", "Moving");
            telemetry.addData("Target FL", frontLeft.getTargetPosition());
            telemetry.addData("Target FR", frontRight.getTargetPosition());
            telemetry.addData("Current FR", frontRight.getCurrentPosition());
            telemetry.addData("Current FL", frontLeft.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
    }

    private void moveElbow(double power, long timeMs) {
        Elbow.setPower(power);
        sleep(timeMs);
        Elbow.setPower(0); // Hold power
    }

    private void armMove(double power, long timeMs) {
        ClipArm.setPower(power);
        sleep(timeMs);
        ClipArm.setPower(0);
    }

    private void Chain(double power, long timeMs) {
        ChainMotor.setPower(power);
        sleep(timeMs);
        ChainMotor.setPower(0);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset to default run mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveForward(double inches, double speed){
        // Reset TRM encoder
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // TRM only
        sleep(100);

        // Calculate target for TRM (forward wheel)
        int trmTarget = -(int)(inches * COUNTS_PER_INCH);  // Negative since TRM reads negative for forward

        telemetry.addData("TRM Target", trmTarget);
        telemetry.update();

        // Set to basic mode
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start driving
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Monitor until TRM reaches target
        while (opModeIsActive() &&
                Math.abs(frontRight.getCurrentPosition()) < Math.abs(trmTarget)) {

            int trmPos = frontRight.getCurrentPosition();
            double progress = Math.abs((double)trmPos / trmTarget * 100);

            telemetry.addData("TRM Position", trmPos);
            telemetry.addData("TRM Target", trmTarget);
            telemetry.addData("Progress", "%.1f%%", progress);
            telemetry.update();

            sleep(10);
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void resetIMU() {
        imu.resetYaw();
    }
// Add these functions to the NewAutonomous class

    private static final double CHAINRATIO = (100.0) / (15.0) * 32;
    private static final double ELBOWRATIO = 75;
    private static final double JERK = .2;
    private static final double A = 13;
    private static final double B = 11;
    private static final double gWStart = .43;

    private void moveChainWithTracking(double power, boolean smoothMotion) {
        final double ACCELERATION_RATE = 0.08;
        final double DECELERATION_RATE = 0.05;
        double currentPower = 0;

        if (smoothMotion) {
            // Smooth power application
            if (Math.abs(currentPower) < Math.abs(power)) {
                currentPower += Math.signum(power) * ACCELERATION_RATE;
            } else if (Math.abs(currentPower) > Math.abs(power)) {
                currentPower -= Math.signum(currentPower) * DECELERATION_RATE;
            }
        } else {
            currentPower = power;
        }

        if (currentPower != 0) {
            ChainMotor.setPower(currentPower);
            ChainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            ChainMotor.setTargetPosition(ChainMotor.getCurrentPosition());
            ChainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ChainMotor.setPower(0.8); // CHAIN_POWER from original
        }
    }

    private void updateElbowPosition(double targetPosition, boolean smooth) {
        final double ELBOW_SMOOTH_FACTOR = 0.2;
        final double ELBOW_POWER = 0.7;

        if (smooth) {
            double currentPosition = Elbow.getCurrentPosition();
            double smoothedPosition = currentPosition +
                    (targetPosition - currentPosition) * ELBOW_SMOOTH_FACTOR;

            Elbow.setTargetPosition((int)smoothedPosition);
        } else {
            Elbow.setTargetPosition((int)targetPosition);
        }

        Elbow.setPower(ELBOW_POWER);
        Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void calculateAndSetGameWristPosition(boolean trace) {
        if (trace) {
            double theta = (ChainMotor.getCurrentPosition()) / 28 / CHAINRATIO;
            double phi = theta + Math.asin((B / A * Math.sin(theta)));
            double newPosition = (theta - phi * 1.2) * (360 / 300) + gWStart;
            GameWrist.setPosition(newPosition);
        }
    }

    private void setRackAndTilter(String position) {
        switch (position.toLowerCase()) {
            case "up":
                Tilter.setPosition(.57);
                break;
            case "down":
                Tilter.setPosition(.4);
                break;
            case "neutral":
                Tilter.setPosition(.51);
                break;
            case "left":
                Rack.setPosition(.2);
                break;
            case "right":
                Rack.setPosition(.725);
                break;
            case "center":
                Rack.setPosition(.56);
                break;
        }
    }

    private void setClipArmPosition(double targetPosition, double power) {
        ClipArm.setTargetPosition((int)(28 * 64 * targetPosition));
        ClipArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ClipArm.setPower(power);
    }

    private void waitForMotorCompletion(DcMotorEx motor) {
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", motor.getTargetPosition());
            telemetry.update();
            sleep(10);
        }
    }

    // Update the existing performClippingSequence to use these new functions
    private void performClippingSequence() {
        ClipHold.setPosition(0.48);
        GameClaw.setPosition(0.4);

        // Move chain motor
        ChainMotor.setTargetPosition((int) (28 * 100 * 0.56));
        ChainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ChainMotor.setPower(0.8);
        waitForMotorCompletion(ChainMotor);

        // Set clip arm position
        setClipArmPosition(0.165, 0.8);
        waitForMotorCompletion(ClipArm);

        // Updated ClipElbow to use motor control
        ClipElbow.setTargetPosition(400);//440
        ClipElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ClipElbow.setPower(0.4);
        waitForMotorCompletion(ClipElbow);

        ClipWrist.setPosition(0.4);

        sleep(500);

        updateElbowPosition(28 * 75 * 0.75, true);
        waitForMotorCompletion(Elbow);

        GameWrist.setPosition(0.62);
        GameTwist.setPosition(0.615);
    }
    // Controls the game claw with standard positions and associated movements

    private void setGameClawPosition(boolean open, boolean withWristAdjust) {
        if (open) {
            // Open position (from dpad_up in TeleOp)
            GameClaw.setPosition(0.85);
            if (withWristAdjust) {
                GameWrist.setPosition(0.62);  // Standard scoring position
                GameTwist.setPosition(0.615); // Standard twist for scoring
            }
        } else {
            // Closed position (from dpad_down in TeleOp)
            GameClaw.setPosition(0.375);
            if (withWristAdjust) {
                GameWrist.setPosition(1.0);   // Pickup position
                GameTwist.setPosition(0.5);   // Neutral twist
            }
        }

        // Allow time for servo movement
        sleep(200);
    }

    /**
     * Simple open/close without wrist adjustment
     */
    private void setGameClawPosition(boolean open) {
        setGameClawPosition(open, false);
    }

    /**
     * Sets the game claw to a specific position
     * @param position Value between 0 and 1
     */
    private void setGameClawPosition(double position) {
        position = Range.clip(position, 0, 1);
        GameClaw.setPosition(position);
        sleep(200);
    }

    /**
     * Sets up game claw for scoring position
     */
    private void setupScoringPosition() {
        GameClaw.setPosition(0.375); // Closed on pixel
        GameWrist.setPosition(0.62); // Scoring position
        GameTwist.setPosition(0.615); // Scoring twist
        sleep(300);
    }

    public void lockClaw() {
        GameWrist.setPosition(0.5);  // Neutral/rest position
        GameTwist.setPosition(0.5);  // Centered position
        GameClaw.setPosition(0.7);   // Closed position to grip
    }

    /**
     * Sets up game claw for pickup position
     */
    private void setupPickupPosition() {
        GameClaw.setPosition(0.85); // Open for pickup
        GameWrist.setPosition(1.0); // Pickup position
        GameTwist.setPosition(0.5); // Neutral twist
        sleep(300);
    }
    private void moveChainToPosition(int targetPosition, double power, boolean smooth) {
        // Constants from TeleOp
        final double ACCELERATION_RATE = 0.08;
        final double DECELERATION_RATE = 0.05;
        final double MIN_POWER = 0.1;
        double currentPower = 0;

        // Set target position
        ChainMotor.setTargetPosition(targetPosition);
        ChainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (smooth) {
            // Accelerate smoothly to target power
            while (opModeIsActive() &&
                    ChainMotor.isBusy() &&
                    currentPower < power) {
                currentPower += ACCELERATION_RATE;
                if (currentPower > power) {
                    currentPower = power;
                }
                ChainMotor.setPower(currentPower);

                // Report progress
                telemetry.addData("Chain Position", ChainMotor.getCurrentPosition());
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Current Power", currentPower);
                telemetry.update();

                sleep(20);  // Short delay for smooth acceleration
            }

            // Run at target power until near destination
            while (opModeIsActive() &&
                    ChainMotor.isBusy() &&
                    Math.abs(targetPosition - ChainMotor.getCurrentPosition()) > 100) {
                telemetry.addData("Chain Position", ChainMotor.getCurrentPosition());
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
                sleep(10);
            }

            // Decelerate as we approach target
            while (opModeIsActive() &&
                    ChainMotor.isBusy() &&
                    currentPower > MIN_POWER) {
                currentPower -= DECELERATION_RATE;
                if (currentPower < MIN_POWER) {
                    currentPower = MIN_POWER;
                }
                ChainMotor.setPower(currentPower);
                telemetry.addData("Chain Position", ChainMotor.getCurrentPosition());
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Current Power", currentPower);
                telemetry.update();
                sleep(20);
            }
        } else {
            // Simple movement without smoothing
            ChainMotor.setPower(power);
            while (opModeIsActive() && ChainMotor.isBusy()) {
                telemetry.addData("Chain Position", ChainMotor.getCurrentPosition());
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
                sleep(10);
            }
        }

        // Final position hold
        ChainMotor.setPower(0.1);  // Hold position
        telemetry.addData("Chain Movement", "Complete");
        telemetry.addData("Final Position", ChainMotor.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Simplified version for quick movements
     */
    private void moveChainToPosition(int targetPosition, double power) {
        moveChainToPosition(targetPosition, power, false);
    }

    /**
     * Moves chain to common scoring positions
     */
    private void moveChainToScoringPosition() {
        // Position from TeleOp scoring sequence
        moveChainToPosition((int)(28 * 100 * 0.56), 0.8, true);
    }

    /**
     * Moves chain to pickup position
     */
    private void moveChainToPickupPosition() {
        moveChainToPosition((int)(28 * 100 * 0.2), 0.8, true);
    }

    /**
     * Returns chain to starting/neutral position
     */
    private void moveChainToNeutral() {
        moveChainToPosition(0, 0.8, true);
    }
    private void performNewScoringSequence() {
        telemetry.addData("Status", "Starting new scoring sequence");
        telemetry.update();

        driveForward(219.6, DRIVE_SPEED);
        driveForward(-108, -DRIVE_SPEED);
        turnToAngle(90,TURN_SPEED);
        driveForward(93, DRIVE_SPEED);
        turnToAngle(70,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(1);
        sleep(300);

        turnToAngle(135,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(0);
        sleep(300);

        turnToAngle(90,TURN_SPEED);
        driveForward(43.2, DRIVE_SPEED);
        turnToAngle(70,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(1);
        sleep(300);

        turnToAngle(135,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(0);
        sleep(300);

        turnToAngle(90,TURN_SPEED);
        driveForward(43.2, DRIVE_SPEED);
        turnToAngle(70,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(1);
        sleep(300);

        turnToAngle(135,TURN_SPEED);
        ExtendRight.setPower(1);
        ExtendLeft.setPower(1);
        sleep(1500);
        GameClaw.setPosition(0);
        sleep(300);

        // 1. Move main Elbow straight up (corrected position)
        telemetry.addData("Status", "Moving main elbow up");
        telemetry.update();
        updateElbowPosition(28 * 75 * 2.25, true);  // Changed to 0.0 for straight up
        waitForMotorCompletion(Elbow);
        sleep(300);

        // 2. Extend chain arm out straight
        telemetry.addData("Status", "Extending chain");
        telemetry.update();
        moveChainToPosition((int)(28 * 100 * 0.6), 0.8, true);  // Using smooth motion
        waitForMotorCompletion(ChainMotor);
        sleep(300);

        // 3. Drive forward 48 inches
        telemetry.addData("Status", "Driving forward");
        telemetry.update();
        driveForward(421, DRIVE_SPEED);
        GameWrist.setPosition(0);
        sleep(300);



        // 5. Extend ClipElbow out (reversed direction)
        telemetry.addData("Status", "Extending clip elbow");
        telemetry.update();
        ClipElbow.setTargetPosition(230);  // Changed to positive value
        ClipElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ClipElbow.setPower(0.4);
        waitForMotorCompletion(ClipElbow);
        sleep(300);




        // 6. Move ClipElbow straight up
        ClipHold.setPosition(0.48);  // Closed position from performClippingSequence
        telemetry.addData("Status", "Moving clip elbow up");
        telemetry.update();
        ClipWrist.setPosition(1.0);
        sleep(1000);
        ClipElbow.setTargetPosition(100);  // Increased from 800
        ClipElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ClipElbow.setPower(1);
        telemetry.addData("Status", "Flipping wrist");
        telemetry.update();
        waitForMotorCompletion(ClipElbow);
        sleep(300);
        telemetry.addData("Status", "Raising clip arm for release");
        telemetry.update();
        sleep(300);
        GameWrist.setPosition(0);

        sleep(300);

        telemetry.update();
        driveForward(-150, -DRIVE_SPEED);
        GameWrist.setPosition(0);
        sleep(300);

        driveForward(-79.2, -DRIVE_SPEED);
        sleep(300);

        // 2. Extend chain arm out straight
        telemetry.addData("Status", "Extending chain");
        telemetry.update();
        moveChainToPosition((int)(28 * 100 * 0.5), 0.8, true);  // Using smooth motion
        waitForMotorCompletion(ChainMotor);
        sleep(300);

        telemetry.addData("Status", "Moving main elbow up");
        telemetry.update();
        updateElbowPosition(28 * 75 * 0.5, true);  // Changed to 0.0 for straight up
        waitForMotorCompletion(Elbow);
        sleep(300);


        telemetry.addData("Status", "Turning 90 degrees");
        telemetry.update();
        turnToAngle(90, TURN_SPEED);
        sleep(300);


        driveForward(860.6, DRIVE_SPEED);
        sleep(300);

        turnToAngle(0, TURN_SPEED);
        sleep(300);

        driveForward(64.8,DRIVE_SPEED);
        sleep(300);

        moveChainToPosition((int)(28 * 100 * 0.75), 0.6, true);  // Using smooth motion
        waitForMotorCompletion(ChainMotor);
        GameTwist.setPosition(0.5);
        sleep(300);

        setGameClawPosition(0);
        sleep(500);

        moveChainToPosition((int)(28 * 100 * 0.3), 0.6, true);  // Using smooth motion
        waitForMotorCompletion(ChainMotor);
        sleep(3000);

        turnToAngle(270, TURN_SPEED);
        sleep(300);
















}}
