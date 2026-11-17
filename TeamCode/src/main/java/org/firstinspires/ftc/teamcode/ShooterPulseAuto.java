package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Carter Shooter Pulse Auto", group="Robot")
public class ShooterPulseAuto extends LinearOpMode {

    // Declare motors and servo

    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotorEx shooter_motor = null;

    private DcMotor intake_motor = null;

    private DcMotor feeder_motor = null;

    private Servo flower_servo = null;
    private ElapsedTime runtime = new ElapsedTime();

    // --- Constants (tune for your robot) ---
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED  = 0.6;

    double setShooterRPM = 0.0;



    @Override
    public void runOpMode() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left_motor   = hardwareMap.get(DcMotor.class, "FL");
        front_right_motor  = hardwareMap.get(DcMotor.class, "FR");
        back_left_motor    = hardwareMap.get(DcMotor.class, "BL");
        back_right_motor   = hardwareMap.get(DcMotor.class, "BR");
        intake_motor       = hardwareMap.get(DcMotor.class, "intake");
        shooter_motor      = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder_motor       = hardwareMap.get(DcMotor.class, "feeder");
        flower_servo       = hardwareMap.get(Servo.class, "flower");



        // --- Motor Directions ---
        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);
        shooter_motor.setDirection(DcMotorEx.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        feeder_motor.setDirection(DcMotor.Direction.FORWARD);

        // --- Encoder Reset ---
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Encoders Reset. Ready to start.");
        telemetry.update();

        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // --- Step 1: Drive forward slightly
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 2.0);  //front of the robot is facing the field wall

        // --- Step 2: Pulsing Motor Action (Non-blocking) ---
        double pulseDuration = 0.3; // How long it stays on/off in seconds
        double totalPulseTime = 2.0; // Total time to pulse back and forth

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < totalPulseTime) {
            // Use the modulo operator (%) to cleanly alternate states
            if ((int)(runtime.seconds() / pulseDuration) % 2 == 0) {
            }

            // Keep telemetry alive and prevent CPU hogging
            telemetry.addData("Pulse Timer", "%.2f / %.2f", runtime.seconds(), totalPulseTime);
            telemetry.update();
        }
        feeder_motor.setPower(0.0); // Safely turn off before moving to Step 3

        // --- Step 3: Spin shooter motors and launch pollen in bursts ---
        setShooterRPM = 2200;
        double targetVelocity = setShooterRPM * 28.0 / 60.0;
        shooter_motor.setVelocity(targetVelocity);
        intake_motor.setPower(1.0); // Keep intake running to stage ammo

        // 1. Wait safely for the shooter to reach up to speed (within 95% threshold)
        while (opModeIsActive() && shooter_motor.getVelocity() < (targetVelocity * 0.95)) {
            telemetry.addData("Shooter Spooling", "Current: %.1f / Target: %.1f", shooter_motor.getVelocity(), targetVelocity);
            telemetry.update();
        }

        // 2. Define your burst parameters
        int totalShots = 3;          // Number of game pieces to fire
        double feedDuration = 0.25;   // How long the feeder pushes forward per shot (seconds)
        double recoveryDelay = 0.2;   // Pause to let the shooter wheel spin back up (seconds)

        for (int shot = 1; shot <= totalShots; shot++) {
            if (!opModeIsActive()) break;

            // --- BURST ON: Feed one piece into the shooter ---
            feeder_motor.setPower(-0.5);

            ElapsedTime burstTimer = new ElapsedTime();
            while (opModeIsActive() && burstTimer.seconds() < feedDuration) {
                telemetry.addData("Firing Burst", "Shot %d of %d", shot, totalShots);
                telemetry.addData("Shooter Velocity", "%.1f", shooter_motor.getVelocity());
                telemetry.update();
            }

            // --- RECOVERY PAUSE: Stop feeder, let shooter catch up on RPM ---
            feeder_motor.setPower(0.0);
            burstTimer.reset();
            while (opModeIsActive() && burstTimer.seconds() < recoveryDelay) {
                telemetry.addData("Recovering RPM", "Waiting for wheel...");
                telemetry.addData("Shooter Velocity", "%.1f", shooter_motor.getVelocity());
                telemetry.update();
            }
        }

        // 3. Clean shutdown of all shooting mechanisms
        shooter_motor.setVelocity(0.0);
        intake_motor.setPower(0.0);
        feeder_motor.setPower(0.0);
//10.28

        // --- Step 4: Drive diagonal 72 inches ---
        encoderDrive(DRIVE_SPEED, 0, 72, 72, 0, 5.0);  //front of the robot is facing the field wall


        // --- Step 5:  Drive forward 64 inches ---
        encoderDrive(DRIVE_SPEED, 64, 64, 64, 64, 5.0);  //front of the robot is facing the field wall


        // --- Step 6: Strafe into parking spot ---
        encoderDrive(DRIVE_SPEED, -32, 32, 32, -32, 5.0);  //front of the robot is facing the field wall





        telemetry.addData("Path", "Complete");
        telemetry.addData("yo did the auto cook", "nah");
        telemetry.update();
        sleep(1000);
    }


    // --- Helper Method for Encoder Drive ---
    public void encoderDrive(double speed,
                             double flInches, double frInches,
                             double blInches, double brInches,
                             double timeoutS) {

        int newFLTarget, newFRTarget, newBLTarget, newBRTarget;

        // Calculate new target positions
        newFLTarget = front_left_motor.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
        newFRTarget = front_right_motor.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
        newBLTarget = back_left_motor.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
        newBRTarget = back_right_motor.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);

        // Set new target positions
        front_left_motor.setTargetPosition(newFLTarget);
        front_right_motor.setTargetPosition(newFRTarget);
        back_left_motor.setTargetPosition(newBLTarget);
        back_right_motor.setTargetPosition(newBRTarget);

        // Switch to RUN_TO_POSITION mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        runtime.reset();
        front_left_motor.setPower(Math.abs(speed));
        front_right_motor.setPower(Math.abs(speed));
        back_left_motor.setPower(Math.abs(speed));
        back_right_motor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (front_left_motor.isBusy() || front_right_motor.isBusy() ||
                        back_left_motor.isBusy() || back_right_motor.isBusy())) {

            telemetry.addData("Running to", "%7d :%7d :%7d :%7d",
                    newFLTarget, newFRTarget, newBLTarget, newBRTarget);
            telemetry.addData("Currently at", "%7d :%7d :%7d :%7d",

                    front_left_motor.getCurrentPosition(),
                    front_right_motor.getCurrentPosition(),
                    back_left_motor.getCurrentPosition(),
                    back_right_motor.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        stopDrive();

        // Return to normal encoder mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    private void stopDrive() {
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }
}

