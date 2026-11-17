package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="8898 First Shooter Auto", group="Robot")
public class FirstShooterAuto8898 extends LinearOpMode {


    // Declare motors and servo
    private DcMotor leftFront  = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront   = null;
    private DcMotor rightBack  = null;
    private DcMotor intake  = null;
    private DcMotorEx outake = null;
    private DcMotorEx outake2 = null;
    private DcMotorEx feeder = null;
    private ServoImplEx intakeBar;
    private ServoImplEx flowerLeft;
    private ServoImplEx flowerRight;
    private boolean flowerDown = false;


    public double outakeSetRPM = 0.0;
    public double outake2SetRPM = 0.0;
    private void encoderTurn(double degrees) {
        double turnSpeed = 0.5;

        // Change these values to match your robot
        double COUNTS_PER_MOTOR_REV = 537.7;
        double DRIVE_GEAR_REDUCTION = 1.0;
        double WHEEL_DIAMETER_INCHES = 4.0;
        double TRACK_WIDTH_INCHES = 15.0;

        double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                        / (WHEEL_DIAMETER_INCHES * Math.PI);

        // Distance each wheel travels during an in-place turn
        double turnCircumference = Math.PI * TRACK_WIDTH_INCHES;
        double turnDistance =
                turnCircumference * Math.abs(degrees) / 360.0;

        int moveCounts = (int) (turnDistance * COUNTS_PER_INCH);

        int leftTarget;
        int rightTarget;

        if (degrees > 0) {
            // Positive angle: turn left
            leftTarget = -moveCounts;
            rightTarget = moveCounts;
        } else {
            // Negative angle: turn right
            leftTarget = moveCounts;
            rightTarget = -moveCounts;
        }

        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + leftTarget);
        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + leftTarget);

        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + rightTarget);
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + rightTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(turnSpeed);
        leftBack.setPower(turnSpeed);
        rightFront.setPower(turnSpeed);
        rightBack.setPower(turnSpeed);

        while (opModeIsActive()
                && (leftFront.isBusy()
                || leftBack.isBusy()
                || rightFront.isBusy()
                || rightBack.isBusy())) {

            telemetry.addData("Turning", degrees);
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    private ElapsedTime runtime = new ElapsedTime();




    final double P = 65.0;
    final double I = 1.5;
    final double D = 1.5;
    final double F = 8.0;




    // --- Constants (tune for your robot) ---
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED  = 0.6;








    @Override
    public void runOpMode() {


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
        flowerLeft = hardwareMap.get(ServoImplEx.class, "flower left");
        flowerRight = hardwareMap.get(ServoImplEx.class, "flower right");


        intakeBar   = hardwareMap.get(ServoImplEx.class, "intakeBar");






        // --- Motor Directions ---
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(DcMotor.Direction.REVERSE);


        feeder.setDirection(DcMotor.Direction.FORWARD);


        outake.setDirection(DcMotorEx.Direction.REVERSE);
        outake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outake.setVelocityPIDFCoefficients(P,I,D,F);


        outake2.setDirection(DcMotorEx.Direction.FORWARD);
        outake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outake2.setVelocityPIDFCoefficients(P,I,D,F);


        flowerLeft.setDirection(Servo.Direction.REVERSE);
        flowerRight.setDirection(Servo.Direction.FORWARD);



        // --- Encoder Reset ---
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        outake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Encoders Reset. Ready to start.");
        telemetry.update();


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();


        // --- Step 1: Drive forward slightly
        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5.0);  //front of the robot is facing the field wall




        // --- Step 2: Drop the Intake
        intakeBar.setPwmEnable();
        intakeBar.setPosition(0.45);
        sleep(500);
        intakeBar.setPwmDisable();




        // --- Step 3: Spin shooter motors and launch pollen ---
        outakeSetRPM = 2100.0;
        outake2SetRPM = 2100.0;
        outake.setVelocity(outakeSetRPM*28.0/60.0);
        outake2.setVelocity(outake2SetRPM*28.0/60.0);
        while (outake.getVelocity() < (outakeSetRPM*28.0/60.0)) {
            //wait for shooter to spin up
        }

        intakeBar.setPwmEnable();
        intakeBar.setPosition(0.45);
        sleep(500);
        intake.setPower(1.0);
        feeder.setPower(1.0);

        sleep(1500);  //delay 1,500 milliseconds

        outake.setVelocity(0.0);
        outake2.setVelocity(0.0);
        intake.setPower(0.0);
        feeder.setPower(0.0);
        intakeBar.setPwmDisable();



        // --- drive to flower
        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5.0);
        encoderTurn(90);
        encoderDrive(DRIVE_SPEED, 16, 16, 16, 16, 5.0);


        //---dispense
        flowerLeft.setPosition(.34);
        flowerRight.setPosition(.34);
        sleep(5500);

        //INTAKE ADD INTAKE CODE


        //drive back
        encoderTurn(-180);
        encoderDrive(DRIVE_SPEED, 16, 16, 16, 16, 5.0);
        encoderTurn(90);
        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5.0);

        //shoot
        outakeSetRPM = 2100.0;
        outake2SetRPM = 2100.0;
        outake.setVelocity(outakeSetRPM*28.0/60.0);
        outake2.setVelocity(outake2SetRPM*28.0/60.0);
        while (outake.getVelocity() < (outakeSetRPM*28.0/60.0)) {
            //wait for shooter to spin up
        }

        intakeBar.setPwmEnable();
        intakeBar.setPosition(0.45);
        sleep(500);
        intake.setPower(1.0);
        feeder.setPower(1.0);

        sleep(1500);  //delay 1,500 milliseconds

        outake.setVelocity(0.0);
        outake2.setVelocity(0.0);
        intake.setPower(0.0);
        feeder.setPower(0.0);
        intakeBar.setPwmDisable();



        // --- Step 4: Drive diagonal 72 inches ---
        encoderDrive(DRIVE_SPEED, -8, 72, 72, -8, 3.0);  //front of the robot is facing the field wall




        // --- Step 5:  Drive forward 47 inches ---
        encoderDrive(DRIVE_SPEED, 47, 47, 47, 47, 3.0);  //front of the robot is facing the field wall




        // --- Step 6: Strafe into parking spot ---
        encoderDrive(DRIVE_SPEED, -30, 30, 30, -30, 3.0);  //front of the robot is facing the field wall


        // --- Step 7:  Drive forward 13 inches ---
        encoderDrive(DRIVE_SPEED, 13, 13, 13, 13, 3.0);  //front of the robot is facing the field wall






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
        newFLTarget = leftFront.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
        newFRTarget = rightFront.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
        newBLTarget = leftBack.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
        newBRTarget = rightBack.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);


        // Set new target positions
        leftFront.setTargetPosition(newFLTarget);
        rightFront.setTargetPosition(newFRTarget);
        leftBack.setTargetPosition(newBLTarget);
        rightBack.setTargetPosition(newBRTarget);


        // Switch to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Start moving
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));


        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {


            telemetry.addData("Running to", "%7d :%7d :%7d :%7d",
                    newFLTarget, newFRTarget, newBLTarget, newBRTarget);
            telemetry.addData("Currently at", "%7d :%7d :%7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }


        // Stop motors
        stopDrive();


        // Return to normal encoder mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sleep(250);
    }


    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}



