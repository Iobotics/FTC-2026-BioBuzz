//imports

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name="CarterJoshuaAuto", group="Linear Opmode")

public class JoshuaCarterAuto extends LinearOpMode {

    //declare motors and other stuff
    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;
    private DcMotor intake_motor = null;
    private ElapsedTime runtime = new ElapsedTime();

    //constants (things about the robot)
    static final double COUNTS_PER_MOTOR_REV = 384.5;  // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED  = 0.6;
    private double MAX_VOLTAGE = 14.0;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() {


        //naming the motors

        front_left_motor = hardwareMap.get(DcMotor.class, "FL");
        front_right_motor = hardwareMap.get(DcMotor.class, "FR");
        back_left_motor = hardwareMap.get(DcMotor.class, "BL");
        back_right_motor = hardwareMap.get(DcMotor.class, "BR");
        intake_motor = hardwareMap.get(DcMotor.class, "intake");

        //directions

        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        //brake

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoder reset

        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //note for when the robot is done initializing

        telemetry.addLine("Encoders Reset. Ready to start.");
        telemetry.update();

        //run the encoder

        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //drive in a square around the field


        encoderDrive(DRIVE_SPEED, 0, 10, 10, 0, 2.0);
        sleep(500);
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5.0);
        sleep(250);
        encoderDrive(TURN_SPEED, -10, 10, -10, 10, 3.0);
        sleep(250);
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5.0);
        sleep(250);
        encoderDrive(TURN_SPEED, -10, 10, -10, 10, 3.0);
        sleep(250);
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5.0);
        sleep(250);
        encoderDrive(TURN_SPEED, -10, 10, -10, 10, 3.0);
        sleep(250);
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5.0);
        sleep(250);
        encoderDrive(TURN_SPEED, 10, -10, 10, -10, 3.0);
        sleep(250);
    }

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
                    (front_left_motor.isBusy() && front_right_motor.isBusy() &&
                            back_left_motor.isBusy() && back_right_motor.isBusy())) {

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







}}