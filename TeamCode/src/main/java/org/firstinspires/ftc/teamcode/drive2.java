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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "8740 Teleop", group = "Iterative OpMode")
public class drive2 extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor Leftfront, Rightfront, Leftback, Rightback;
    private DcMotor Intake, Feeder;
    private DcMotorEx Shooter;
    private ServoImplEx intakebar, rampleft, rampright;

    final double TICKS_PER_REV = 28.0;
    final double P = 15.0, I = 2.5, D = 1.0, F = 1.5;
    final double READY_FRACTION = 0.95;

    double targetShooterRPM = 2600;
    double currentShooterRPM = 0;

    final double[] DIST_IN = {  30,   50,   70,   90,  110 };
    final double[] RPM_TBL = {2350, 2800, 3200, 3300, 3400 };

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    boolean exposureSet = false;

    final String CAMERA_NAME = "webcam";
    final double TAG_SIZE_IN = 3.25;
    final int RED_FIRST_ID = 30;
    final int BLUE_FIRST_ID = 38;
    final int CELLS_PER_ALLIANCE = 2;
    final int TAGS_PER_CELL = 4;

    final int EXPOSURE_MS = 6;
    final int GAIN = 250;

    final double CAMERA_LEFT_OFFSET_IN = 6.0;

    final double CAMERA_FWD_OFFSET_IN = 0.0;

    final double CAMERA_YAW_LEFT_DEG = 0.0;

    final double CAMERA_HEIGHT_IN = 10.0;
    final double CAMERA_TILT_UP_DEG = 0.0;

    final double MIN_TARGET_HEIGHT_IN = -999;

    final double BEARING_TOL = 1.5;
    final double AIM_OFFSET_DEG = 0.0;

    final double TURN_GAIN = 0.02;
    final double MAX_TURN = 0.35;
    final double MIN_TURN = 0.08;
    final double SEARCH_TURN = 0.20;
    final double LOST_HOLD_S = 0.30;
    final double ALIGNED_HOLD_S = 0.25;
    final double AUTO_TIMEOUT_S = 6.0;
    final double SPIN_HOLD_S = 3.0;
    final double CANCEL_STICK = 0.25;

    final double TURN_SIGN = -1.0;


    boolean blueAlliance = true;
    boolean autoAlignActive = false;
    String alignStatus = "IDLE";
    double autoStartTime = 0;
    double lastTargetTime = 0;
    double alignedSince = -1;
    double spinHoldUntil = 0;
    double searchDir = -1;
    int lockedCell = -1;

    private static class HiveTarget {
        int cell;
        double bearing, range, height;
        int tagsSeen;
    }

    boolean isReversed = false;
    boolean lastGamepad1A = false;
    boolean alliancePrev = false;
    boolean alignBtnPrev = false;
    boolean yPrev, bPrev, aPrev, upPrev, leftPrev, rightPrev;

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

        AprilTagLibrary.Builder lib = new AprilTagLibrary.Builder();
        for (int id = RED_FIRST_ID; id < BLUE_FIRST_ID + CELLS_PER_ALLIANCE * TAGS_PER_CELL; id++) {
            lib.addTag(id, "HIVE " + id, TAG_SIZE_IN, DistanceUnit.INCH);
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(lib.build())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        aprilTag.setDecimation(2);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
    }

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
        } catch (Exception e) {
        }
        exposureSet = true;
    }

    @Override
    public void init_loop() {
        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
        }
        alliancePrev = gamepad1.y;

        setManualExposure(EXPOSURE_MS, GAIN);

        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Camera state", portal.getCameraState());
        telemetry.addData("# detections", aprilTag.getDetections().size());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double now = runtime.seconds();
        setManualExposure(EXPOSURE_MS, GAIN);

        handleServos();
        handleToggles();
        handleShooterTargetAdjust();

        List<AprilTagDetection> allTags = aprilTag.getDetections();
        HiveTarget target = findHiveTarget(allTags, autoAlignActive ? lockedCell : -1);

        boolean alignBtn = gamepad1.x;
        if (alignBtn && !alignBtnPrev) {
            if (autoAlignActive) {
                autoAlignActive = false;
                alignStatus = "CANCELLED";
            } else {
                autoAlignActive = true;
                alignStatus = "SEARCHING";
                autoStartTime = now;
                lastTargetTime = now;
                alignedSince = -1;
                lockedCell = -1;
            }
        }
        alignBtnPrev = alignBtn;

        boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > CANCEL_STICK
                || Math.abs(gamepad1.left_stick_y) > CANCEL_STICK
                || Math.abs(gamepad1.right_stick_x) > CANCEL_STICK;
        if (autoAlignActive && sticksMoved) {
            autoAlignActive = false;
            alignStatus = "CANCELLED";
        }

        double twist = 0;

        if (autoAlignActive) {
            if (now - autoStartTime > AUTO_TIMEOUT_S) {
                autoAlignActive = false;
                alignStatus = "TIMEOUT";
            } else if (target != null) {
                lockedCell = target.cell;
                lastTargetTime = now;

                double bearingErr = target.bearing - AIM_OFFSET_DEG;
                if (Math.abs(bearingErr) > 0.01) searchDir = Math.signum(bearingErr);

                boolean headingOk = Math.abs(bearingErr) < BEARING_TOL;
                alignStatus = "ALIGNING";

                if (!headingOk) {
                    twist = TURN_SIGN * Range.clip(bearingErr * TURN_GAIN, -MAX_TURN, MAX_TURN);
                    if (Math.abs(twist) < MIN_TURN) twist = Math.copySign(MIN_TURN, twist);
                }

                if (headingOk) {
                    if (alignedSince < 0) alignedSince = now;
                    if (now - alignedSince >= ALIGNED_HOLD_S) {

                        targetShooterRPM = Math.round(rpmForDistance(target.range) / 10.0) * 10.0;
                        spinHoldUntil = now + SPIN_HOLD_S;
                        autoAlignActive = false;
                        alignStatus = "ALIGNED";
                        gamepad1.rumble(400);
                        twist = 0;
                    }
                } else {
                    alignedSince = -1;
                }
            } else {

                alignedSince = -1;
                alignStatus = "SEARCHING";
                if (now - lastTargetTime > LOST_HOLD_S) {
                    twist = TURN_SIGN * searchDir * SEARCH_TURN;
                }
            }
        }

        if (autoAlignActive) {

            driveMecanum(0, 0, twist);
        } else {

            double drive = gamepad1.left_stick_x;
            double strafe = gamepad1.left_stick_y;
            double manualTwist = gamepad1.right_stick_x;
            if (isReversed) {
                drive = -drive;
                strafe = -strafe;
            }
            driveMecanum(drive, strafe, manualTwist);
        }

        currentShooterRPM = Shooter.getVelocity() * 60.0 / TICKS_PER_REV;

        double activeRPM = (autoAlignActive && target != null)
                ? rpmForDistance(target.range) : targetShooterRPM;

        boolean reverseAll = gamepad1.b;
        boolean shootHeld = gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed;
        boolean feedOnly = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean outtake = gamepad1.right_bumper || gamepad2.right_trigger_pressed;
        boolean intakeIn = gamepad1.right_trigger_pressed || gamepad2.right_bumper;

        double intakePower = 0.0;
        double feederPower = 0.0;
        double shooterRPM = 0.0;

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

        boolean preSpin = autoAlignActive || now < spinHoldUntil;
        if (preSpin && !shootHeld && !reverseAll) {
            shooterRPM = activeRPM;
        }

        Intake.setPower(intakePower);
        Feeder.setPower(feederPower);
        Shooter.setVelocity(shooterRPM * TICKS_PER_REV / 60.0);

        double shooterAmps = Shooter.getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Orientation", isReversed ? "REVERSED (Back is Front)" : "NORMAL");
        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");

        telemetry.addLine("--- Auto-align (tap X) ---");
        telemetry.addData("Align status", alignStatus);
        if (target != null) {
            telemetry.addData("Target cell", target.cell + " (" + target.tagsSeen + " tags)");
            telemetry.addData("Bearing (deg)", "%5.1f", target.bearing);
            telemetry.addData("Range (in)", "%5.1f", target.range);
            telemetry.addData("Est. height (in)", "%5.1f", target.height);
        } else {
            telemetry.addData("Target", "none visible");
        }

        telemetry.addLine("--- Vision ---");
        telemetry.addData("Camera state", portal.getCameraState());
        telemetry.addData("# detections (all)", allTags.size());
        for (AprilTagDetection d : allTags) {
            if (d.metadata != null && d.ftcPose != null) {
                telemetry.addData("Tag " + d.id, "brg %.1f  rng %.1f", d.ftcPose.bearing, d.ftcPose.range);
            } else {
                telemetry.addData("Tag " + d.id, "NO metadata (ignored)");
            }
        }

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

    private HiveTarget findHiveTarget(List<AprilTagDetection> detections, int preferCell) {
        int base = blueAlliance ? BLUE_FIRST_ID : RED_FIRST_ID;
        double[] bearingSum = new double[CELLS_PER_ALLIANCE];
        double[] rangeSum = new double[CELLS_PER_ALLIANCE];
        double[] heightSum = new double[CELLS_PER_ALLIANCE];
        int[] count = new int[CELLS_PER_ALLIANCE];
        double tilt = Math.toRadians(CAMERA_TILT_UP_DEG);
        double yaw = Math.toRadians(CAMERA_YAW_LEFT_DEG);

        for (AprilTagDetection d : detections) {
            if (d.metadata == null || d.ftcPose == null) continue;
            int idx = d.id - base;
            if (idx < 0 || idx >= CELLS_PER_ALLIANCE * TAGS_PER_CELL) continue;
            int cell = idx / TAGS_PER_CELL;

            double cx = d.ftcPose.x, cy = d.ftcPose.y;
            double right = cx * Math.cos(yaw) - cy * Math.sin(yaw) - CAMERA_LEFT_OFFSET_IN;
            double fwd = cx * Math.sin(yaw) + cy * Math.cos(yaw) + CAMERA_FWD_OFFSET_IN;

            bearingSum[cell] += -Math.toDegrees(Math.atan2(right, fwd));
            rangeSum[cell] += Math.sqrt(right * right + fwd * fwd + d.ftcPose.z * d.ftcPose.z);
            heightSum[cell] += CAMERA_HEIGHT_IN
                    + d.ftcPose.y * Math.sin(tilt) + d.ftcPose.z * Math.cos(tilt);
            count[cell]++;
        }

        int best = -1;
        if (preferCell >= 0 && preferCell < CELLS_PER_ALLIANCE && count[preferCell] > 0) {
            best = preferCell;
        } else {
            double bestHeight = -Double.MAX_VALUE;
            for (int c = 0; c < CELLS_PER_ALLIANCE; c++) {
                if (count[c] == 0) continue;
                double h = heightSum[c] / count[c];
                if (h < MIN_TARGET_HEIGHT_IN) continue;
                if (h > bestHeight) {
                    bestHeight = h;
                    best = c;
                }
            }
        }
        if (best < 0) return null;

        HiveTarget t = new HiveTarget();
        t.cell = best;
        t.tagsSeen = count[best];
        t.bearing = bearingSum[best] / count[best];
        t.range = rangeSum[best] / count[best];
        t.height = heightSum[best] / count[best];
        return t;
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

    private void handleServos() {
        if (gamepad1.dpad_down) intakebar.setPosition(0.75);
        if (gamepad1.dpad_up) intakebar.setPosition(0.25);

        if (gamepad1.dpad_left) {
            rampleft.setPosition(1);
            rampright.setPosition(1);
        }
        if (gamepad1.dpad_right) {
            rampleft.setPosition(0);
            rampright.setPosition(0);
        }
    }

    private void handleToggles() {
        if (gamepad1.a && !lastGamepad1A) {
            isReversed = !isReversed;
        }
        lastGamepad1A = gamepad1.a;

        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
            lockedCell = -1;
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
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist),
        };

        double max = 1.0;
        for (double s : speeds) max = Math.max(max, Math.abs(s));
        for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

        Rightfront.setPower(speeds[0]);
        Rightback.setPower(speeds[1]);
        Leftfront.setPower(speeds[2]);
        Leftback.setPower(speeds[3]);
    }
}
