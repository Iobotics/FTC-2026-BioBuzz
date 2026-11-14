package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "8740 Teleop", group = "Iterative OpMode")
public class drive2 extends OpMode {

    // ---------------- Hardware ----------------
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor Leftfront, Rightfront, Leftback, Rightback;
    private DcMotor Intake, Feeder;
    private DcMotorEx Shooter;
    private ServoImplEx intakebar, rampleft, rampright;

    // ---------------- Shooter ----------------
    final double TICKS_PER_REV = 28.0;
    final double P = 15.0, I = 2.5, D = 1.0, F = 1.5;
    final double READY_FRACTION = 0.95;   // feed only once shooter is at 95% of target

    double targetShooterRPM = 2600;     // manual target (gamepad2 adjusts this)
    double currentShooterRPM = 0;

    // ---------------- Vision ----------------
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    boolean exposureSet = false;

    final String CAMERA_NAME = "webcam";  // must match Robot Configuration exactly
    final int BLUE_TAG = 20;
    final int RED_TAG = 24;

    // Auto-aim tuning
    final double TURN_GAIN = 0.02;
    final double MAX_TURN = 0.35;
    final double MIN_TURN = 0.08;
    final double BEARING_TOL = 1.5;       // degrees
    final double SEARCH_TURN = 0.20;      // slow spin when X held and no tag visible
    // If the robot turns AWAY from the tag, change this to +1.0
    final double TURN_SIGN = -1.0;

    // Distance (inches) -> RPM table
    final double[] DIST_IN = {  30,   50,   70,   90,  110 };
    final double[] RPM_TBL = {2350, 2800, 3200, 3300, 3400 };

    boolean blueAlliance = true;
    boolean aiming = false;
    double tagBearing = 0;
    double tagRange = 0;

    // ---------------- Button edge detection ----------------
    boolean isReversed = false;
    boolean lastGamepad1A = false;
    boolean alliancePrev = false;
    boolean yPrev, bPrev, xPrev, aPrev, upPrev, leftPrev, downPrev, rightPrev;

    @Override
    public void init() {
        Leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        Leftback = hardwareMap.get(DcMotor.class, "Leftback");
        Rightback = hardwareMap.get(DcMotor.class, "Rightback");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Feeder = hardwareMap.get(DcMotor.class, "feeder");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        intakebar = hardwareMap.get(ServoImplEx.class, "intakeBar");
        intakebar.setPwmEnable();
        rampleft = hardwareMap.get(ServoImplEx.class, "rampleft");
        rampleft.setPwmEnable();
        rampright = hardwareMap.get(ServoImplEx.class, "rampright");
        rampright.setPwmEnable();

        rampleft.setDirection(Servo.Direction.REVERSE);
        rampright.setDirection(Servo.Direction.FORWARD);

        Leftfront.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.REVERSE);
        Leftback.setDirection(DcMotor.Direction.FORWARD);
        Rightback.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Feeder.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotor[] brakeMotors = {Leftfront, Rightfront, Leftback, Rightback, Intake, Feeder};
        for (DcMotor m : brakeMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter.setVelocityPIDFCoefficients(P, I, D, F);

        initVision();

        telemetry.addData("Status", "Initialized");
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                // If you have calibration for your webcam, add:
                // .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        // Lower decimation = longer detection range but slower FPS. Try 2 or 3.
        aprilTag.setDecimation(2);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // less USB bandwidth
                .addProcessor(aprilTag)
                .build();
    }

    /** Short exposure + high gain reduces motion blur. Only works once the camera is streaming. */
    private void setManualExposure(int exposureMs, int gainValue) {
        if (portal == null || exposureSet) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);

            GainControl gain = portal.getCameraControl(GainControl.class);
            gain.setGain(Range.clip(gainValue, gain.getMinGain(), gain.getMaxGain()));

            exposureSet = true;
        } catch (Exception e) {
            // Some webcams don't support manual control; fall back to auto
            exposureSet = true;
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
        }
        alliancePrev = gamepad1.y;

        setManualExposure(6, 250);

        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Camera state", portal.getCameraState());
        telemetry.addData("Manual exposure set", exposureSet);
        telemetry.addData("# detections", aprilTag.getDetections().size());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        setManualExposure(6, 250);  // no-op if already applied

        handleServos();
        handleToggles();
        handleShooterTargetAdjust();

        // ---------- Vision ----------
        List<AprilTagDetection> allTags = aprilTag.getDetections();
        AprilTagDetection tag = findTag(allTags, blueAlliance ? BLUE_TAG : RED_TAG);
        if (tag != null) {
            tagBearing = tag.ftcPose.bearing;
            tagRange = tag.ftcPose.range;
        }

        boolean xHeld = gamepad1.x;
        aiming = xHeld && tag != null;

        // ---------- Driving inputs ----------
        // NOTE: kept as in your original (left_stick_x -> drive, left_stick_y -> strafe).
        // Most robots use drive = -left_stick_y, strafe = left_stick_x; leave as-is if it drives correctly.
        double drive = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double twist = gamepad1.right_stick_x;

        if (aiming) {
            if (Math.abs(tagBearing) < BEARING_TOL) {
                twist = 0;
            } else {
                twist = TURN_SIGN * Range.clip(tagBearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
                if (Math.abs(twist) < MIN_TURN) {
                    twist = Math.copySign(MIN_TURN, twist);
                }
            }
        } else if (xHeld) {
            // X held but no tag in view: slowly spin to search for it
            twist = -TURN_SIGN * SEARCH_TURN;
        }

        if (isReversed) {
            drive = -drive;
            strafe = -strafe;
        }

        driveMecanum(drive, strafe, twist);

        // ---------- Shooter / intake / feeder ----------
        currentShooterRPM = Shooter.getVelocity() * 60.0 / TICKS_PER_REV;

        // Auto-aim sets the RPM from distance without overwriting your manual target
        double activeRPM = aiming ? rpmForDistance(tagRange) : targetShooterRPM;

        boolean reverseAll = gamepad1.b;
        boolean shootHeld = gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed;
        boolean feedOnly = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean outtake = gamepad1.right_bumper || gamepad2.right_trigger_pressed;
        boolean intakeIn = gamepad1.right_trigger_pressed || gamepad2.right_bumper;

        double intakePower = 0.0;
        double feederPower = 0.0;
        double shooterRPM = 0.0;

        // Priority (highest first): reverseAll (gamepad1.b) > shoot > feed only > outtake > intake
        if (reverseAll) {
            intakePower = -1.0;
            feederPower = -1.0;
            shooterRPM = -activeRPM;
        } else if (shootHeld) {
            shooterRPM = activeRPM;
            boolean atSpeed = Math.abs(currentShooterRPM) >= Math.abs(activeRPM) * READY_FRACTION;
            if (atSpeed) {
                intakePower = 1.0;
                feederPower = 0.8;
            }
        } else if (feedOnly) {
            feederPower = 0.5;
        } else if (outtake) {
            intakePower = -0.40;
        } else if (intakeIn) {
            intakePower = 1.0;
        }

        // Pre-spin the shooter while auto-aiming, even before the shoot trigger is pulled
        if (aiming && !shootHeld) {
            shooterRPM = activeRPM;
        }

        Intake.setPower(intakePower);
        Feeder.setPower(feederPower);
        Shooter.setVelocity(shooterRPM * TICKS_PER_REV / 60.0);

        // ---------- Telemetry ----------
        double shooterAmps = Shooter.getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Orientation", isReversed ? "REVERSED (Back is Front)" : "NORMAL");
        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");

        telemetry.addLine("--- Vision ---");
        telemetry.addData("Camera state", portal.getCameraState());
        telemetry.addData("# detections (all)", allTags.size());
        for (AprilTagDetection d : allTags) {
            telemetry.addData("Seen tag " + d.id,
                    d.metadata != null ? "metadata OK" : "NO metadata (ignored)");
        }
        telemetry.addData("Looking for tag", blueAlliance ? BLUE_TAG : RED_TAG);
        telemetry.addData("Tag visible", tag != null);
        telemetry.addData("Auto aim (hold X)", aiming ? "ACTIVE" : (xHeld ? "SEARCHING" : "off"));
        telemetry.addData("Tag bearing", "%4.1f", tagBearing);
        telemetry.addData("Tag range", "%4.1f", tagRange);

        telemetry.addLine("--- Shooter ---");
        telemetry.addData("Manual target RPM", targetShooterRPM);
        telemetry.addData("Active target RPM", activeRPM);
        telemetry.addData("Current RPM", currentShooterRPM);
        telemetry.addData("Shooter Amps", "%4.2f", shooterAmps);
        telemetry.addData("Intake Power", "%4.2f", intakePower);
        telemetry.addData("Feeder Power", "%4.2f", feederPower);
        telemetry.addData("IntakeBar Position", "%4.2f", intakebar.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }

    // ======================= Helpers =======================

    private void handleServos() {
        if (gamepad1.dpad_down) intakebar.setPosition(0.75);
        if (gamepad1.dpad_up) intakebar.setPosition(0.25);

        if (gamepad1.dpad_left) rampleft.setPosition(1);
        if (gamepad1.dpad_right) rampleft.setPosition(0);
        if (gamepad1.dpad_left) rampright.setPosition(1);
        if (gamepad1.dpad_right) rampright.setPosition(0);
}

    private void handleToggles() {
        if (gamepad1.a && !lastGamepad1A) {
            isReversed = !isReversed;
        }
        lastGamepad1A = gamepad1.a;

        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
        }
        alliancePrev = gamepad1.y;
    }

    private void handleShooterTargetAdjust() {
        if (gamepad2.b && !bPrev) targetShooterRPM -= 50;
        bPrev = gamepad2.b;

        if (gamepad2.y && !yPrev) targetShooterRPM += 50;
        yPrev = gamepad2.y;

        if (gamepad2.a && !aPrev) targetShooterRPM += 100;
        aPrev = gamepad2.a;

        //if (gamepad2.x && !xPrev) targetShooterRPM = -targetShooterRPM;
        //xPrev = gamepad2.x;

        if (gamepad2.dpad_up && !upPrev) targetShooterRPM = 2600;
        upPrev = gamepad2.dpad_up;

        if (gamepad2.dpad_left && !leftPrev) targetShooterRPM = 2800;
        leftPrev = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !rightPrev) targetShooterRPM = 2500;
        rightPrev = gamepad2.dpad_right;

        targetShooterRPM = Range.clip(targetShooterRPM, -6000, 6000);
    }

    private void driveMecanum(double drive, double strafe, double twist) {
        double[] speeds = {
                (drive + strafe + twist),   // Rightfront
                (drive - strafe - twist),   // Rightback
                (drive - strafe + twist),   // Leftfront
                (drive + strafe - twist),   // Leftback
        };

        double max = 1.0;
        for (double s : speeds) max = Math.max(max, Math.abs(s));
        for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

        Rightfront.setPower(speeds[0]);
        Rightback.setPower(speeds[1]);
        Leftfront.setPower(speeds[2]);
        Leftback.setPower(speeds[3]);
    }

    private AprilTagDetection findTag(List<AprilTagDetection> detections, int id) {
        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == id) {
                return d;
            }
        }
        return null;
    }

    private double rpmForDistance(double distIn) {
        int last = DIST_IN.length - 1;
        if (distIn <= DIST_IN[0]) return RPM_TBL[0];
        if (distIn >= DIST_IN[last]) return RPM_TBL[last];
        for (int i = 0; i < last; i++) {
            if (distIn <= DIST_IN[i + 1]) {
                double t = (distIn - DIST_IN[i]) / (DIST_IN[i + 1] - DIST_IN[i]);
                return RPM_TBL[i] + t * (RPM_TBL[i + 1] - RPM_TBL[i]);
            }
        }
        return RPM_TBL[last];
    }
}
