package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="USE THIS ONE FOR NOW", group="Robot")
public class Badbadauto extends LinearOpMode {

    /* Declare drive members ONLY */
    private DcMotor Leftfront = null;
    private DcMotor Rightfront = null;
    private DcMotor Leftback = null;
    private DcMotor Rightback = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate COUNTS_PER_INCH for drive train (GoBILDA 312 RPM Yellowjacket = 537.7 PPR, 96mm wheel)
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             = 0.6;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        Leftfront  = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        Leftback   = hardwareMap.get(DcMotor.class, "Leftback");
        Rightback  = hardwareMap.get(DcMotor.class, "Rightback");

        // Motor directions matching TeleOp
        Leftfront.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.FORWARD);
        Leftback.setDirection(DcMotor.Direction.REVERSE);
        Rightback.setDirection(DcMotor.Direction.REVERSE);

        Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized & Ready");
        telemetry.update();

        // Wait for Driver Station START button
        waitForStart();

        // Drive forward 24 inches to leave starting position
        encoderDrive(DRIVE_SPEED, 24, 24, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /*
     * Method to perform relative move for drive wheels based on encoder counts
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int lfTarget, rfTarget, lbTarget, rbTarget;

        if (opModeIsActive()) {

            lfTarget = Leftfront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rfTarget = Rightfront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lbTarget = Leftback.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rbTarget = Rightback.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            Leftfront.setTargetPosition(lfTarget);
            Rightfront.setTargetPosition(rfTarget);
            Leftback.setTargetPosition(lbTarget);
            Rightback.setTargetPosition(rbTarget);

            Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Leftfront.setPower(Math.abs(speed));
            Rightfront.setPower(Math.abs(speed));
            Leftback.setPower(Math.abs(speed));
            Rightback.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (Leftfront.isBusy() && Rightfront.isBusy() && Leftback.isBusy() && Rightback.isBusy())) {

                telemetry.addData("Targets", "LF:%7d RF:%7d LB:%7d RB:%7d", lfTarget, rfTarget, lbTarget, rbTarget);
                telemetry.addData("Current", "LF:%7d RF:%7d LB:%7d RB:%7d",
                        Leftfront.getCurrentPosition(), Rightfront.getCurrentPosition(),
                        Leftback.getCurrentPosition(), Rightback.getCurrentPosition());
                telemetry.update();
            }

            Leftfront.setPower(0);
            Rightfront.setPower(0);
            Leftback.setPower(0);
            Rightback.setPower(0);

            Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}
