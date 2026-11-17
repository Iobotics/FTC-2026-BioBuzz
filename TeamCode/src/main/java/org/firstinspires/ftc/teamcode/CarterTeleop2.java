package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="CarterTeleop", group="Linear Opmode")
public class CarterTeleop2 extends LinearOpMode {

    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotorEx shooter_motor = null;
    private DcMotor intake_motor = null;
    private DcMotor feeder_motor = null;

    private Servo flower_servo = null;

    double targetShooterRPM = 2100;
    double currentShooterRPM = 0;
    double setShooterRPM = 0.0;

    final double P = 45.0;
    final double I = 0.65;
    final double D = 1.2;
    final double F = 9.0;

    private ElapsedTime feederPulseTimer = new ElapsedTime();
    private boolean isPulsingFeeder = false;
    private final double FEEDER_PULSE_DURATION_MS = 300;

    private final ElapsedTime timer = new ElapsedTime();
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

    final double CAMERA_HEIGHT_IN = 15.5;
    final double CAMERA_TILT_UP_DEG = 30.0;
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
    final double CANCEL_STICK = 0.25;
    final double TURN_SIGN = 1.0;

    boolean blueAlliance = true;
    boolean autoAlignActive = false;
    String alignStatus = "IDLE";
    double autoStartTime = 0;
    double lastTargetTime = 0;
    double alignedSince = -1;
    double searchDir = -1;
    int lockedCell = -1;

    private static class HiveTarget {
        int cell;
        double bearing, range, height;
        int tagsSeen;
    }

    @Override
    public void runOpMode() {

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

        initVision();

        boolean isReversed = false;
        boolean aPrev = false;
        boolean dpad_downPrev = false;
        boolean dpad_upPrev = false;
        boolean bPrev = false;
        boolean alliancePrev = false;
        boolean alignBtnPrev = false;

        while (opModeInInit()) {
            if (gamepad1.dpad_right && !alliancePrev) {
                blueAlliance = !blueAlliance;
            }
            alliancePrev = gamepad1.dpad_right;

            setManualExposure();

            telemetry.addData("Alliance (dpad right to toggle)", blueAlliance ? "BLUE" : "RED");
            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("# detections", aprilTag.getDetections().size());
            telemetry.update();
        }

        waitForStart();
        resetRuntime();
        timer.reset();

        while (opModeIsActive()) {
            double now = timer.seconds();
            setManualExposure();

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;

            if (gamepad1.dpad_left && !aPrev) {
                isReversed = !isReversed;
            }
            aPrev = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !alliancePrev) {
                blueAlliance = !blueAlliance;
                lockedCell = -1;
            }
            alliancePrev = gamepad1.dpad_right;

            if (gamepad2.dpad_down && !dpad_downPrev) {
                targetShooterRPM = targetShooterRPM - 50;
            }
            dpad_downPrev = gamepad2.dpad_down;

            if (gamepad2.dpad_up && !dpad_upPrev) {
                targetShooterRPM = targetShooterRPM + 50;
            }
            dpad_upPrev = gamepad2.dpad_up;

            List<AprilTagDetection> allTags = aprilTag.getDetections();
            HiveTarget target = findHiveTarget(allTags, autoAlignActive ? lockedCell : -1);

            boolean alignBtn = gamepad1.a;
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

            double autoTwist = 0;

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
                        autoTwist = TURN_SIGN * Range.clip(bearingErr * TURN_GAIN, -MAX_TURN, MAX_TURN);
                        if (Math.abs(autoTwist) < MIN_TURN) autoTwist = Math.copySign(MIN_TURN, autoTwist);
                    }

                    if (headingOk) {
                        if (alignedSince < 0) alignedSince = now;
                        if (now - alignedSince >= ALIGNED_HOLD_S) {
                            autoAlignActive = false;
                            alignStatus = "ALIGNED";
                            gamepad1.rumble(400);
                            autoTwist = 0;
                        }
                    } else {
                        alignedSince = -1;
                    }
                } else {
                    alignedSince = -1;
                    alignStatus = "SEARCHING";
                    if (now - lastTargetTime > LOST_HOLD_S) {
                        autoTwist = TURN_SIGN * searchDir * SEARCH_TURN;
                    }
                }
            }

            if (autoAlignActive) {
                drive = 0;
                strafe = 0;
                twist = autoTwist;
            } else if (isReversed) {
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

            if (gamepad1.left_trigger > 0.1) {
                setShooterRPM = targetShooterRPM;
            } else {
                setShooterRPM = 0;
            }

            if (gamepad1.y) {
                setShooterRPM = -targetShooterRPM;

            }

            if (gamepad1.left_bumper) {
                feeder_pwr = 0.5;
            } else if (gamepad1.right_bumper) {
                feeder_pwr = -0.5;
            }

            if (gamepad1.b && !bPrev) {
                isPulsingFeeder = true;
                feederPulseTimer.reset();
            }
            bPrev = gamepad1.b;

            if (isPulsingFeeder) {
                if (feederPulseTimer.milliseconds() < FEEDER_PULSE_DURATION_MS) {
                    feeder_pwr = 0.5;
                } else {
                    isPulsingFeeder = false;
                }
            }

            if (gamepad1.right_trigger > 0.1) {
                intake_pwr = 0.5;
            } else if (gamepad1.x) {
                intake_pwr = -0.5;
            }

            front_left_motor.setPower(frontLeftPower);
            front_right_motor.setPower(frontRightPower);
            back_left_motor.setPower(backLeftPower);
            back_right_motor.setPower(backRightPower);
            intake_motor.setPower(intake_pwr);
            feeder_motor.setPower(feeder_pwr);

            double targetTicksPerSec = (setShooterRPM * 28.0) / 60.0;
            double currentShooterRPM = (shooter_motor.getVelocity()/28*60);
            shooter_motor.setVelocity(targetTicksPerSec);

            telemetry.addData("Shooter Target RPM: ", targetShooterRPM);
            telemetry.addData("Shooter Current RPM: ", currentShooterRPM);
            telemetry.addData("Drive Is Reversed", "%s",
                    isReversed);
            telemetry.addData("Alliance (dpad right)", blueAlliance ? "BLUE" : "RED");
            telemetry.addData("Align status (A)", alignStatus);
            if (target != null) {
                telemetry.addData("Target cell", target.cell + " (" + target.tagsSeen + " tags)");
                telemetry.addData("Bearing (deg)", "%5.1f", target.bearing);
                telemetry.addData("Range (in)", "%5.1f", target.range);
            } else {
                telemetry.addData("Target", "none visible");
            }
            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("# detections (all)", allTags.size());
            telemetry.addData("YOU'VE GOT THIS! C", "%s", "");
            telemetry.addData("Hi, if you're reading this you're probably the operator, remember to read stuff out for your driver OR ELSE thanks <3", "%s", "");
            telemetry.addData("hi operator this is a note from carter ur glorious, handsome, short, smart, beautiful programmer (im most likely talking to myself) but be sure to shout at the top of your lungs, 'PARK PARK PARK PARK'", "%s", "");
            telemetry.update();

        }

        if (portal != null) {
            portal.close();
        }
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

    private void setManualExposure() {
        if (portal == null || exposureSet) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);

            GainControl gain = portal.getCameraControl(GainControl.class);
            gain.setGain(Range.clip(GAIN, gain.getMinGain(), gain.getMaxGain()));
        } catch (Exception e) {
        }
        exposureSet = true;
    }

    private HiveTarget findHiveTarget(List<AprilTagDetection> detections, int preferCell) {
        int base = blueAlliance ? BLUE_FIRST_ID : RED_FIRST_ID;
        double[] bearingSum = new double[CELLS_PER_ALLIANCE];
        double[] rangeSum = new double[CELLS_PER_ALLIANCE];
        double[] heightSum = new double[CELLS_PER_ALLIANCE];
        int[] count = new int[CELLS_PER_ALLIANCE];
        double tilt = Math.toRadians(CAMERA_TILT_UP_DEG);

        for (AprilTagDetection d : detections) {
            if (d.metadata == null || d.ftcPose == null) continue;
            int idx = d.id - base;
            if (idx < 0 || idx >= CELLS_PER_ALLIANCE * TAGS_PER_CELL) continue;
            int cell = idx / TAGS_PER_CELL;

            bearingSum[cell] += d.ftcPose.bearing;
            rangeSum[cell] += d.ftcPose.range;
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
}
