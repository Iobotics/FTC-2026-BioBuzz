package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="CarterTeleop", group="Linear Opmode")
public class CarterTeleop2 extends LinearOpMode {

    // declare and initialize 7 DcMotors.
    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotorEx shooter_motor = null;
    private DcMotor intake_motor = null;
    private DcMotor feeder_motor = null;

    private Servo flower_servo = null;

    // declare shooter control variables
    double targetShooterRPM = 2000;
    double currentShooterRPM = 0;
    double setShooterRPM = 0.0;

    final double P = 45.0;
    final double I = 0.65;
    final double D = 1.2;
    final double F = 9.0;

    // Non-blocking pulse variables for the feeder
    private ElapsedTime feederPulseTimer = new ElapsedTime();
    private boolean isPulsingFeeder = false;
    private final double FEEDER_PULSE_DURATION_MS = 300; // Adjust this duration (in milliseconds) for a perfect feed

    @Override
    public void runOpMode() {

        // Name strings must match up with the config on the Robot Controller app.
        front_left_motor    = hardwareMap.get(DcMotor.class, "FL");
        front_right_motor   = hardwareMap.get(DcMotor.class, "FR");
        back_left_motor     = hardwareMap.get(DcMotor.class, "BL");
        back_right_motor    = hardwareMap.get(DcMotor.class, "BR");
        intake_motor        = hardwareMap.get(DcMotor.class, "intake");
        shooter_motor       = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder_motor        = hardwareMap.get(DcMotor.class, "feeder");
        flower_servo        = hardwareMap.get(Servo.class, "flower");

        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);
        shooter_motor.setDirection(DcMotorEx.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        feeder_motor.setDirection(DcMotor.Direction.FORWARD);
        flower_servo.setDirection(Servo.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter_motor.setVelocityPIDFCoefficients(P, I, D, F);

        boolean isReversed = false;
        boolean aPrev = false;
        boolean a2Prev = false;
        boolean dpad_downPrev = false;
        boolean dpad_upPrev = false;
        boolean bPrev = false;

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;

            // Toggle reverse on A press (edge detection).
            if (gamepad1.a && !aPrev) {
                isReversed = !isReversed;
            }
            aPrev = gamepad1.a;

            // Toggle reverse on gamepad 2 A press.
            if (gamepad2.a && !a2Prev) {
                isReversed = !isReversed;
            }
            a2Prev = gamepad2.a;

            // Decrease target shooter RPM by 50 on dpad down press.
            if (gamepad2.dpad_down && !dpad_downPrev) {
                targetShooterRPM = targetShooterRPM - 50;
            }
            dpad_downPrev = gamepad2.dpad_down;

            // Increase target shooter RPM by 50 on dpad up press.
            if (gamepad2.dpad_up && !dpad_upPrev) {
                targetShooterRPM = targetShooterRPM + 50;
            }
            dpad_upPrev = gamepad2.dpad_up;

            if (isReversed) {
                drive = -drive;
                strafe = -strafe;
            }

            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs(speeds[0]);
            for (int i = 0; i < speeds.length; i++) {
                if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
            }

            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            double frontLeftPower = speeds[0];
            double frontRightPower = speeds[1];
            double backLeftPower = speeds[2];
            double backRightPower = speeds[3];

            double intake_pwr = 0;
            double feeder_pwr = 0;
            setShooterRPM = 0.0;

            // Shooter control via Left Trigger
            if (gamepad1.left_trigger > 0.1) {
                setShooterRPM = targetShooterRPM;
            } else {
                setShooterRPM = 0;
            }

            // Manual feeder overrides (Bumpers)
            if (gamepad1.left_bumper) {
                feeder_pwr = 0.5;
            } else if (gamepad1.right_bumper) {
                feeder_pwr = -0.5;
            }

            // Automated Non-Blocking Feeder Pulse on B button press
            if (gamepad1.b && !bPrev) {
                isPulsingFeeder = true;
                feederPulseTimer.reset();
            }
            bPrev = gamepad1.b;

            // If a pulse sequence is active, it overrides manual feeder power
            if (isPulsingFeeder) {
                if (feederPulseTimer.milliseconds() < FEEDER_PULSE_DURATION_MS) {
                    feeder_pwr = 0.5; // Power during the pulse
                } else {
                    isPulsingFeeder = false; // Pulse timed out
                }
            }

            // Intake control (Right Trigger / X Button)
            if (gamepad1.right_trigger > 0.1) {
                intake_pwr = 0.5;
            } else if (gamepad1.x) {
                intake_pwr = -0.5;
            }

            // --- SEND POWER TO ALL MOTORS ---
            front_left_motor.setPower(frontLeftPower);
            front_right_motor.setPower(frontRightPower);
            back_left_motor.setPower(backLeftPower);
            back_right_motor.setPower(backRightPower);
            intake_motor.setPower(intake_pwr);
            feeder_motor.setPower(feeder_pwr);

            // Shooter DcMotorEx Velocity Control
            // Note: DcMotorEx .setVelocity() takes encoder ticks per second.
            // Formula assuming a standard encoder (e.g., 28 ticks per revolution for HD Hex):
            // Ticks Per Second = (RPM * TicksPerRev) / 60.0
            double targetTicksPerSec = (setShooterRPM * 28.0) / 60.0;
            double currentShooterRPM = (shooter_motor.getVelocity()/28*60);
            shooter_motor.setVelocity(targetTicksPerSec);


            telemetry.addData("Shooter Target RPM: ", targetShooterRPM);
            telemetry.addData("Shooter Current RPM: ", currentShooterRPM);
            telemetry.addData("Drive Is Reversed", "%s",
                    isReversed);
            telemetry.addData("YOU'VE GOT THIS! C", "%s", "");
            telemetry.addData("Hi, if you're reading this you're probably the operator, remember to read stuff out for your driver OR ELSE thanks <3", "%s", "");
            telemetry.addData("hi operator this is a note from carter ur glorious, handsome, short, smart, beautiful programmer (im most likely talking to myself) but be sure to shout at the top of your lungs, 'PARK PARK PARK PARK'", "%s", "");
            telemetry.update();

        }
    }}