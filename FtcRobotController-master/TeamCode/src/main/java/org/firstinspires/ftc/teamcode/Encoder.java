package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="Shooter (tip side) (use this one)", group="Robot")
public class Encoder extends LinearOpMode {

    /* Declare drive and subsystem members */
    private DcMotor Leftfront = null;
    private DcMotor Rightfront = null;
    private DcMotor Leftback = null;
    private DcMotor Rightback = null;
    private DcMotor Intake = null;
    private DcMotor Feeder = null;
    private DcMotorEx Shooter = null;
    final double P = 15.0;
    final double I = 2.5;
    final double D = 1.0;
    final double F = 1.5;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate COUNTS_PER_INCH for drive train (GoBILDA 312 RPM Yellowjacket = 537.7 PPR, 96mm wheel)
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize drive & mechanism hardware
        Leftfront  = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        Leftback   = hardwareMap.get(DcMotor.class, "Leftback");
        Rightback  = hardwareMap.get(DcMotor.class, "Rightback");
        Intake     = hardwareMap.get(DcMotor.class, "intake");
        Feeder     = hardwareMap.get(DcMotor.class, "feeder");
        Shooter    = hardwareMap.get(DcMotorEx.class, "Shooter");

        // Motor directions matching TeleOp (Drive.java)
        Leftfront.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.FORWARD);
        Leftback.setDirection(DcMotor.Direction.REVERSE);
        Rightback.setDirection(DcMotor.Direction.FORWARD);


        Intake.setDirection(DcMotor.Direction.REVERSE);
        Feeder.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setDirection(DcMotorEx.Direction.REVERSE);

        Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Feeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set RunModes
        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter.setVelocityPIDFCoefficients(P,I,D,F);

        telemetry.addData("Status", "Initialized & Ready");
        telemetry.update();

        // Wait for Driver Station START button

        waitForStart();

        // Step 2: Shoot (Spins up to 3500 RPM, waits until 95% speed, then feeds for 1.5s)
        shootAndFeed(1.5);

        // Step 1: Go forward 6 inches
        encoderDrive(DRIVE_SPEED, -4, -4, -4, -4, 5.0);

        // Step 3: Turn Left 180° so FRONT (Intake side) faces downfield
        //encoderDrive(TURN_SPEED, 40.0, -40.0, 40.0, -40.0, 4.0);

        // Step 4: Strafe Right 49.5 inches to align intake with lane
        encoderDrive(DRIVE_SPEED, 52, -52, -52, 52, 5.0);

        // Step 5: Drive FORWARD 6 inches while Intaking & Feeding
        driveWithIntake(DRIVE_SPEED, 6.0, 5.0);
        sleep(1000);

        // Step 6: Go back 10 inches
        encoderDrive(DRIVE_SPEED, -10.0, -10.0, -10.0, -10.0, 5.0);

        // Step 7: Strafe right 20 inches to align intake with lane
        encoderDrive(DRIVE_SPEED, -20.0, 20.0, 20.0, -20.0, 5.0);

        // Step 8: Turn 90° Right
        encoderDrive(TURN_SPEED, 20.0, -20.0, 20.0, -20.0, 5.0);

        // Step 9: Strafe right 76 inches to align intake with lane
        encoderDrive(DRIVE_SPEED, 76.0, -76.0, -76.0, 76.0, 5.0);

        encoderDrive(DRIVE_SPEED, 25, 25, 25, 25, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /*
     * Method to perform relative move for 4 individual drive wheels based on encoder counts
     */
    public void encoderDrive(double speed,
                             double flInches, double frInches,
                             double blInches, double brInches,
                             double timeoutS) {
        int lfTarget, rfTarget, lbTarget, rbTarget;

        if (opModeIsActive()) {

            lfTarget = Leftfront.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
            rfTarget = Rightfront.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
            lbTarget = Leftback.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
            rbTarget = Rightback.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);

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
                    (Leftfront.isBusy() || Rightfront.isBusy() || Leftback.isBusy() || Rightback.isBusy())) {

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

    /*
     * Overloaded helper for 2-parameter tank-style drive calls
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        encoderDrive(speed, leftInches, rightInches, leftInches, rightInches, timeoutS);
    }

    /*
     * Helper method to strafe sideways with Mecanum wheels
     */
    public void strafeDrive(double speed, double strafeInches, double timeoutS) {
        if (!opModeIsActive()) return;

        int counts = (int)(strafeInches * COUNTS_PER_INCH);

        int lfTarget = Leftfront.getCurrentPosition() + counts;
        int rfTarget = Rightfront.getCurrentPosition() - counts;
        int lbTarget = Leftback.getCurrentPosition() - counts;
        int rbTarget = Rightback.getCurrentPosition() + counts;

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
                (Leftfront.isBusy() || Rightfront.isBusy() || Leftback.isBusy() || Rightback.isBusy())) {
            telemetry.addData("Strafing", "LF:%7d RF:%7d", lfTarget, rfTarget);
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

    /*
     * Helper method to set shooter to 3300 RPM, wait until 95% speed, then feed and intake
     */
    public void shootAndFeed(double feedTimeS) {
        if (!opModeIsActive()) return;

        double targetRPM = 3300.0;
        double targetTPS = targetRPM * 28.0 / 60.0; // Convert RPM to ticks per second
        double minTPS = targetTPS * 0.95;            // 95% speed threshold (3135 RPM)

        // Enable velocity control on shooter with PIDF coefficients
        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter.setVelocityPIDFCoefficients(P, I, D, F);
        Shooter.setVelocity(targetTPS);

        // Phase 1: Wait until shooter reaches 95% of 3300 RPM
        runtime.reset();
        while (opModeIsActive() && Shooter.getVelocity() < minTPS && runtime.seconds() < 5.0) {
            double currentRPM = Shooter.getVelocity() * 60.0 / 28.0;
            double currentAmps = Shooter.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Shooter Current RPM", "%4.2f", currentRPM);
            telemetry.addData("Shooter Target RPM", "%4.2f", targetRPM);
            telemetry.addData("Shooter Amps", "%4.2f", currentAmps);
            telemetry.addData("Status", "Spinning up...");
            telemetry.update();
        }

        // Phase 2: Feed ball into active shooter (Run Feeder AND Intake)
        Feeder.setPower(0.2);
        Intake.setPower(0.8);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < feedTimeS) {
            double currentRPM = Shooter.getVelocity() * 60.0 / 28.0;
            double currentAmps = Shooter.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Shooter Current RPM", "%4.2f", currentRPM);
            telemetry.addData("Shooter Target RPM", "%4.2f", targetRPM);
            telemetry.addData("Shooter Amps", "%4.2f", currentAmps);
            telemetry.addData("Status", "Feeding & Launching...");
            telemetry.update();
        }

        // Phase 3: Stop Shooter, Feeder, and Intake
        Shooter.setVelocity(0.0);
        Feeder.setPower(0);
        Intake.setPower(0);
    }

    public void shootAndFeed(double unusedSpinUpTimeS, double feedTimeS) {
        shootAndFeed(feedTimeS);
    }

    /*
     * Helper method to run intake and feeder to collect balls
     */
    public void runIntake(double runTimeS) {
        if (!opModeIsActive()) return;

        Intake.setPower(0.8);
        Feeder.setPower(0.8);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < runTimeS) {
            telemetry.addData("Intake", "Collecting balls...");
            telemetry.update();
        }

        Intake.setPower(0);
        Feeder.setPower(0);
    }

    /*
     * Helper method to drive forward while running intake and feeder
     */
    public void driveWithIntake(double speed, double forwardInches, double timeoutS) {
        if (!opModeIsActive()) return;

        int counts = (int)(forwardInches * COUNTS_PER_INCH);

        int lfTarget = Leftfront.getCurrentPosition() + counts;
        int rfTarget = Rightfront.getCurrentPosition() + counts;
        int lbTarget = Leftback.getCurrentPosition() + counts;
        int rbTarget = Rightback.getCurrentPosition() + counts;

        Leftfront.setTargetPosition(lfTarget);
        Rightfront.setTargetPosition(rfTarget);
        Leftback.setTargetPosition(lbTarget);
        Rightback.setTargetPosition(rbTarget);

        Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Turn on Intake and Feeder while driving
        Intake.setPower(0.8);
        Feeder.setPower(0.8);

        runtime.reset();
        Leftfront.setPower(Math.abs(speed));
        Rightfront.setPower(Math.abs(speed));
        Leftback.setPower(Math.abs(speed));
        Rightback.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Leftfront.isBusy() || Rightfront.isBusy() || Leftback.isBusy() || Rightback.isBusy())) {
            telemetry.addData("Driving + Intaking", "Target: %d", lfTarget);
            telemetry.update();
        }

        // Stop drive, intake, and feeder
        Leftfront.setPower(0);
        Rightfront.setPower(0);
        Leftback.setPower(0);
        Rightback.setPower(0);
        Intake.setPower(0);
        Feeder.setPower(0);

        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }
}
