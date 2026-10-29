package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Carter Shooter Auto", group="Robot")
public class ShooterAuto extends LinearOpMode {

    // Declare motors and servo

    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotorEx shooter_motor = null;

    private DcMotor intake_motor = null;

    private DcMotor feeder_motor = null;
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
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);  //front of the robot is facing the field wall

        feeder_motor.setPower(1.0);
        sleep(300);
        feeder_motor.setPower(-1.0);

        // --- Step 3: Spin shooter motors and launch pollen ---
        setShooterRPM = 3200;
        shooter_motor.setVelocity(setShooterRPM*28.0/60.0);
        //Converts RPM into Ticks per Second at the Encoder
        while (shooter_motor.getVelocity() < (setShooterRPM*28.0/60.0)) {
            //wait for motor to get to speed
        }
        intake_motor.setPower(1.0);
        feeder_motor.setPower(1.0);
        sleep(1500);  //delay 1,500 milliseconds
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

