package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private ServoImplEx intakeBar;

    private double intake_power;


    private boolean isReversed = false;
    private boolean lastA = false;

    public double outakeSetRPM = 0.0;
    public double outake2SetRPM = 0.0;


    private ElapsedTime runtime = new ElapsedTime();

    private double servoTimer;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront    = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        outake  = hardwareMap.get(DcMotorEx.class, "outake");
        outake2  = hardwareMap.get(DcMotorEx.class, "outake");

        intakeBar   = hardwareMap.get(ServoImplEx.class, "intakeBar");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outake.setDirection(DcMotorEx.Direction.FORWARD);
        outake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outake2.setDirection(DcMotorEx.Direction.REVERSE);
        outake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

    }

    @Override
    public void loop() {
        if  (gamepad1.x) {
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
        if (gamepad1.b) {
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.27);
            runtime.reset();
            servoTimer = runtime.seconds();
            while (servoTimer < 1.0) {
                //wait for Servo
                servoTimer = runtime.seconds();
            }
            //intakeBar.setPwmDisable();
        }

        //Code for running Intake
        if (gamepad1.right_trigger > 0.0 && gamepad1.left_trigger < 0.01) {
            intake_power = 1.0;
            intakeBar.setPwmEnable();
            intakeBar.setPosition(0.27);

        }

        //Code when no triggers are pressed
        if (gamepad1.right_trigger < 0.01 && gamepad1.left_trigger < 0.01) {
            intake_power = 0.0;
            outakeSetRPM = 0.0;
            outake2SetRPM = 0.0;
            intakeBar.setPwmDisable();
        }

        //Code to run the Outake
        if(gamepad1.left_trigger > 0.0 && gamepad1.right_trigger < 0.01) {
            outakeSetRPM = 1600.0;
            outake2SetRPM = 1600.0;
            if ((outake.getVelocity() * 60.0 / 28.0) < outakeSetRPM) {
                //wait for shooter to spin up
            }
            if ((outake.getVelocity() * 60.0 / 28.0) >= outakeSetRPM) {
                intake_power = 1.0;
                intakeBar.setPwmEnable();
                intakeBar.setPosition(0.0);

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

        //double intake_power = gamepad1.right_trigger;



        if (isReversed) {
            drive = -drive;
            strafe = -strafe;
            twist = -twist;
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
        double outakeCurrentRPM = outake.getVelocity()*60/28;

        double servoPosition = intakeBar.getPosition();

        telemetry.addData("Outake Current RPM", "%4.2f", outakeCurrentRPM);
        telemetry.addData("Servo Position", "%4.2f", servoPosition);
        telemetry.addData("Intake Power", "%4.2f" , intake_power);
        telemetry.update();
    }
}