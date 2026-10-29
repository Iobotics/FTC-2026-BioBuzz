
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="CarterTeleop", group="Linear Opmode")
public class CarterTeleop2 extends LinearOpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize 7 DcMotors.
    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotorEx shooter_motor = null;

    private DcMotor intake_motor = null;

    private DcMotor feeder_motor = null;
    //declare shooter control variables
    double targetShooterRPM = 2000;
    double currentShooterRPM = 0;
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

        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);
        shooter_motor.setDirection(DcMotorEx.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        feeder_motor.setDirection(DcMotor.Direction.REVERSE);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean isReversed = false;
        boolean aPrev = false;
        boolean a2Prev = false;
        boolean dpad_downPrev = false;
        boolean dpad_upPrev = false;

        waitForStart();
        resetRuntime();

        while (opModeIsActive()){
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            double drive  = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist  = -gamepad1.right_stick_x;

            /*
             * If we had a gyro and wanted to do field-oriented control, here
             * is where we would implement it.
             *
             * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
             * coordinate (strafe, drive), and we just rotate it by the gyro
             * reading minus the offset that we read in the init() method.
             * Some rough pseudocode demonstrating:
             *
             * if Field Oriented Control:
             *     get gyro heading
             *     subtract initial offset from heading
             *     convert heading to radians (if necessary)
             *     new strafe = strafe * cos(heading) - drive * sin(heading)
             *     new drive  = strafe * sin(heading) + drive * cos(heading)
             *
             * If you want more understanding on where these rotation formulas come
             * from, refer to
             * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
             */

            // Toggle reverse on A press (edge detection).
            // The Prev thingy prevents button from continuously activating without being released
            if (gamepad1.a && !aPrev) {
                isReversed = !isReversed;
            }
            aPrev = gamepad1.a;

            // Toggle reverse on gamepad 2 A press.
            // The Prev thingy prevents button from continuously activating without being released
            if (gamepad2.a && !a2Prev) {
                isReversed = !isReversed;
            }
            a2Prev = gamepad2.a;

            // Decrease target shooter RPM by 50 on dpad down press.
            // The Prev thingy prevents button from continuously activating without being released
            if (gamepad2.dpad_down && !dpad_downPrev) {
                targetShooterRPM = targetShooterRPM-50;
            }
            dpad_downPrev = gamepad2.dpad_down;

            // Increase target shooter RPM by 50 on dpad up press.
            // The Prev thingy prevents button from continuously activating without being released
            if (gamepad2.dpad_up && !dpad_upPrev) {
                targetShooterRPM = targetShooterRPM+50;
            }
            dpad_upPrev = gamepad2.dpad_up;

            if (isReversed) {
                drive = -drive;
                strafe = -strafe;
            }

            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.


            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }
/*              (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)

                */

            // If and only if the maximum is outside the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }


            //double multiplier = isReversed ? -1.0 : 1.0;

            double frontLeftPower = speeds[0];
            double frontRightPower = speeds[1];
            double backLeftPower = speeds[2];
            double backRightPower = speeds[3];



            double intake_pwr = 0;
            //double shooter_pwr = 0;
            double feeder_pwr = 0;
            setShooterRPM = 0.0;

            if (gamepad1.left_bumper) {
                feeder_pwr = 0.5;
            }

            if (gamepad1.left_trigger > 0.1) {
                setShooterRPM = targetShooterRPM;
            }

            else {
                setShooterRPM = 0;
            }

            if (gamepad1.right_bumper) {
                feeder_pwr = -0.5;
            }
            if (gamepad1.right_trigger_pressed) {
                intake_pwr = 0.5;
            }

            if (gamepad1.x) {
                intake_pwr = -0.5;

            if (gamepad1.b) {
                setShooterRPM = -targetShooterRPM;

            }

            else {
                setShooterRPM = 0;
                }

            }
            // apply the calculated values to the motors.


            front_left_motor.setPower(frontLeftPower);
            front_right_motor.setPower(frontRightPower);
            back_left_motor.setPower(backLeftPower);
            back_right_motor.setPower(backRightPower);
            intake_motor.setPower(intake_pwr);
            shooter_motor.setVelocity(setShooterRPM*28.0/60.0);  //converts RPM into Ticks per Second at the Encoder
            feeder_motor.setPower(feeder_pwr);
            currentShooterRPM = shooter_motor.getVelocity()*60/28;

            // Prints stuff for your eyeholes to see during match, assuming said eyeholes and attached brain are capable of reading
            telemetry.addData("Target RPM", targetShooterRPM);
            telemetry.addData("Current RPM", currentShooterRPM);
            telemetry.addData("Drive Is Reversed", "%s",
                    isReversed);
            telemetry.addData("YOU'VE GOT THIS! C", "%s","");
            telemetry.addData("Hi, if you're reading this you're probably the operator, remember to read stuff out for your driver OR ELSE thanks <3", "%s","");
            telemetry.addData("hi operator this is a note from carter ur glorious, handsome, short, smart, beautiful programmer (im most likely talking to myself) but be sure to shout at the top of your lungs, 'PARK PARK PARK PARK'", "%s", "" );
            telemetry.update();
        }
    }}


