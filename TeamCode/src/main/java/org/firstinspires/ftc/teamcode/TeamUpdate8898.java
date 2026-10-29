package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="TeamUpdate8898", group="Linear Opmode")
public class TeamUpdate8898 extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor leftFront  = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront   = null;
    private DcMotor rightBack  = null;
    private DcMotor intake  = null;
    private DcMotorEx outake = null;
    private DcMotorEx outake2 = null;
    private DcMotorEx feeder = null;
    private ServoImplEx intakeBar;


    private ServoImplEx swiper;

    private double intake_power;

    private double feeder_power;


    private boolean isReversed = false;
    private boolean lastA = false;

    public double outakeSetRPM = 0.0;
    public double outake2SetRPM = 0.0;


    private ElapsedTime runtime = new ElapsedTime();

    private double servoTimer;

    final double P = 65.0;
    final double I = 1.5;
    final double D = 1.5;
    final double F = 8.0;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront    = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        outake  = hardwareMap.get(DcMotorEx.class, "outake");
        outake2  = hardwareMap.get(DcMotorEx.class, "outake2");

        intakeBar   = hardwareMap.get(ServoImplEx.class, "intakeBar");
        swiper = hardwareMap.get(ServoImplEx.class, "swiper");



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        outake.setDirection(DcMotorEx.Direction.REVERSE);
        outake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outake.setVelocityPIDFCoefficients(P,I,D,F);

        outake2.setDirection(DcMotorEx.Direction.FORWARD);
        outake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outake2.setVelocityPIDFCoefficients(P,I,D,F);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        runtime.reset();

    }

    @Override
    public void loop() {

        intake_power = 0.0;
        outakeSetRPM = 0.0;
        outake2SetRPM = 0.0;
        feeder_power = 0.0;

        //Code for running the swiper
        if (gamepad1.y) {
            swiper.setPosition(0.15);
            for (double i = 0.15; i < 0.65 ; i+= 0.001) {
                swiper.setPosition(i);
            }
            for (double i = 0.65; i> 0.15 ; i-= 0.001) {
                swiper.setPosition(i);
            }
        }

        //Code for resetting swiper
        if (!gamepad1.y) {
            swiper.setPosition(0.0);
        }

        /*if  (gamepad1.x) {
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.0);
            runtime.reset();
            servoTimer = runtime.seconds();
            while (servoTimer < 1.0) {
                //wait for Servo
                servoTimer = runtime.seconds();
            }
            //intakeBar.setPwmDisable();
        }

         */


        /*if (gamepad1.b) {
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.45);
            runtime.reset();
            servoTimer = runtime.seconds();
            while (servoTimer < 1.0) {
                //wait for Servo
                servoTimer = runtime.seconds();
            }
            //intakeBar.setPwmDisable();
        }

         */

        //Code for running Intake
        if (gamepad1.right_trigger > 0.0 && gamepad1.left_trigger < 0.01) {
            intake_power = 1.0;
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.45);

        }

        //Code for back spinning Intake to push out balls
        if (gamepad1.right_bumper) {
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.45);
            intake_power = -0.65;
        }

        //Code for when Intake is not running
        if (!gamepad1.right_bumper && gamepad1.right_trigger < 0.01) {
            intakeBar.setPwmDisable();
        }



        //Code to run the Outake
        if(gamepad1.left_trigger > 0.0 && gamepad1.right_trigger < 0.01) {
            outakeSetRPM = 2200.0;
            outake2SetRPM = 2200.0;
            if (outake.getVelocity() < (outakeSetRPM*28.0/60.0)) {
                //wait for shooter to spin up
            }
            if (outake.getVelocity() >= (outakeSetRPM*28.0/60.0)) {
                intakeBar.setPwmEnable();
                intakeBar.setPosition(0.45);
                intake_power = 1.0;
                feeder_power = 1.0;

            }
        }

        if (gamepad1.a && !lastA) {
            isReversed = !isReversed;
        }

        lastA = gamepad1.a;

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;
        twist = 0.75*twist;

        //double intake_power = gamepad1.right_trigger;



        if (isReversed) {
            drive = -drive;
            strafe = -strafe;
            //twist = -twist;
        }

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

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe + twist),
                (drive - strafe - twist),
                (drive + strafe - twist),
        };



        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }



        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }


        // apply the calculated values to the motors.
        leftFront.setPower(speeds[0]);
        leftBack.setPower(speeds[1]);
        rightFront.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);

        intake.setPower(intake_power);
        outake.setVelocity(outakeSetRPM * 28.0/60.0);
        outake2.setVelocity(outake2SetRPM * 28.0/60.0);
        feeder.setPower(feeder_power);
        double outakeCurrentRPM = outake.getVelocity()*60/28;

        double outakeAmps = outake.getCurrent(CurrentUnit.AMPS);
        double outake2Amps = outake2.getCurrent(CurrentUnit.AMPS);

        double servoPosition = intakeBar.getPosition();

        telemetry.addData("Outake RPM", "%4.2f", outakeCurrentRPM);
        telemetry.addData("Outake 1 Amps","%4.2f",outakeAmps);
        telemetry.addData("Outake 2 Amps","%4.2f", outake2Amps);
        telemetry.addData("Servo Position", "%4.2f", servoPosition);
        telemetry.addData("Intake Power", "%4.2f" , intake_power);
        telemetry.update();
    }
}