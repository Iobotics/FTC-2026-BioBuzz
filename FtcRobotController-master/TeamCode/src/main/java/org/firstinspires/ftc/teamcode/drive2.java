package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="8740 tele2", group="Iterative OpMode")

public class drive2 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Leftfront = null;
    private DcMotor Rightfront = null;
    private DcMotor Leftback = null;
    private DcMotor Rightback = null;
    private DcMotor Intake = null;
    private DcMotor Feeder = null;
    private ServoImplEx intakebar = null;
    boolean isReversed = false;

    boolean lastGamepad1A = false;
    boolean yPrev = false;
    boolean bPrev = false;
    boolean xPrev = false;
    boolean aPrev = false;
    boolean upPrev = false;
    boolean leftPrev = false;
    boolean downPrev = false;
    boolean rightPrev = false;
    boolean alliancePrev = false;

    private DcMotorEx Shooter = null;

    double shooterSetRPM = 0.0;

    double intake_power = 0.0;

    double feeder_power = 0.0;
    double targetShooterRPM = 3300.0;

    double currentShooterRPM = 0;
    final double P = 15.0;
    final double I = 2.5;
    final double D = 1.0;
    final double F = 1.5;

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    final String CAMERA_NAME = "Webcam 1";
    final int BLUE_TAG = 20;
    final int RED_TAG = 24;
    final double TURN_GAIN = 0.02;
    final double MAX_TURN = 0.35;
    final double MIN_TURN = 0.08;
    final double BEARING_TOL = 1.5;
    final double[] DIST_IN = {  30,   50,   70,   90,  110 };
    final double[] RPM_TBL = {2350, 2800, 3200, 3300, 3400 };

    boolean blueAlliance = true;
    boolean aiming = false;
    double tagBearing = 0;
    double tagRange = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        Leftback = hardwareMap.get(DcMotor.class, "Leftback");
        Rightback = hardwareMap.get(DcMotor.class, "Rightback");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Feeder = hardwareMap.get(DcMotor.class, "feeder");
        intakebar = hardwareMap.get(ServoImplEx.class, "intakeBar");
        intakebar.setPwmEnable();
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        Leftfront.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.REVERSE);
        Leftback.setDirection(DcMotor.Direction.FORWARD);
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

        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter.setVelocityPIDFCoefficients(P,I,D,F);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
        }
        alliancePrev = gamepad1.y;
        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_down) {
            intakebar.setPosition(0.75);
        }
        if (gamepad1.dpad_up) {
            intakebar.setPosition(0.25);
        }

        if (gamepad1.a && !lastGamepad1A) {
            isReversed = !isReversed;
        }
        lastGamepad1A = gamepad1.a;

        if (gamepad1.y && !alliancePrev) {
            blueAlliance = !blueAlliance;
        }
        alliancePrev = gamepad1.y;

        double drive = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double twist = gamepad1.right_stick_x;

        if(gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed) {
            intake_power = 1.0;
            shooterSetRPM = 0.0;
            feeder_power = 0.0;
        }
        if(gamepad2.right_bumper && !gamepad2.right_trigger_pressed) {
            intake_power = 1.0;
            shooterSetRPM = 0.0;
            feeder_power = 0.0;
        }

        if(gamepad1.right_bumper && !gamepad1.left_trigger_pressed) {
            intake_power = -0.40;
            shooterSetRPM = 0.0;
            feeder_power = 0.0;
        }

        if(gamepad2.right_trigger_pressed && !gamepad2.right_bumper) {
            intake_power = -0.40;
            shooterSetRPM = 0.0;
            feeder_power = 0.0;
        }

        if(gamepad1.left_bumper && !gamepad1.left_trigger_pressed) {
            intake_power = 0.0;
            shooterSetRPM = 0.0;
            feeder_power = 0.5;
        }
        if(gamepad2.left_bumper && !gamepad2.left_trigger_pressed) {
            intake_power = 0.0;
            shooterSetRPM = 0.0;
            feeder_power = 0.5;
        }

        if(gamepad1.left_trigger_pressed && !gamepad1.right_trigger_pressed) {
            shooterSetRPM = 3300.0;
            if(Shooter.getVelocity()*60.0/28.0 < targetShooterRPM * 0.95) {
                intake_power = 0.0;
                feeder_power = 0.0;
            }
            if(Shooter.getVelocity()*60.0/28.0 >= targetShooterRPM * 0.95) {
                intake_power = 1.0;
                feeder_power = 1.0;
            }
        }

        if(gamepad2.left_trigger_pressed && !gamepad2.left_bumper) {
            shooterSetRPM = 3300.0;
            if(Shooter.getVelocity()*60.0/28.0 < targetShooterRPM * 0.95) {
                intake_power = 0.0;
                feeder_power = 0.0;
            }
            if(Shooter.getVelocity()*60.0/28.0 >= targetShooterRPM * 0.95) {
                intake_power = 1.0;
                feeder_power = 1.0;
            }
        }

        if(!gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed && !gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad2.right_trigger_pressed && !gamepad2.left_trigger_pressed && !gamepad2.right_bumper && !gamepad2.left_bumper) {
            shooterSetRPM = 0.0;
            intake_power = 0.0;
            feeder_power = 0.0;
        }

        if (gamepad2.b && !bPrev) {
            targetShooterRPM = targetShooterRPM-50;
        }
        bPrev = gamepad2.b;

        if (gamepad2.y && !yPrev) {
            targetShooterRPM = targetShooterRPM+50;
        }
        yPrev = gamepad2.y;

        if (gamepad2.a && !aPrev) {
            targetShooterRPM = targetShooterRPM+100;
        }
        aPrev = gamepad2.a;
        if (targetShooterRPM > 6000) {
            targetShooterRPM = 6000;
        }
        if (targetShooterRPM < -6000) {
            targetShooterRPM = -6000;
        }

        if (gamepad2.x && !xPrev) {
            targetShooterRPM = -targetShooterRPM;
        }
        xPrev = gamepad2.x;

        if (gamepad2.dpad_up && !upPrev) {
            targetShooterRPM = 3400;
        }
        upPrev = gamepad2.dpad_up;

        if (gamepad2.dpad_left && !leftPrev) {
            targetShooterRPM = 3300;
        }
        leftPrev = gamepad2.dpad_left;

        if (gamepad2.dpad_down && !downPrev) {
            targetShooterRPM = 3200;
        }
        downPrev = gamepad2.dpad_down;

        if (gamepad2.dpad_right && !rightPrev){
            targetShooterRPM = 2350;
        }
        rightPrev = gamepad2.dpad_right;

        AprilTagDetection tag = findTag(blueAlliance ? BLUE_TAG : RED_TAG);
        aiming = gamepad1.x && tag != null;
        if (tag != null) {
            tagBearing = tag.ftcPose.bearing;
            tagRange = tag.ftcPose.range;
        }
        if (aiming) {
            if (Math.abs(tagBearing) < BEARING_TOL) {
                twist = 0;
            } else {
                twist = -Range.clip(tagBearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
                if (Math.abs(twist) < MIN_TURN) {
                    twist = Math.copySign(MIN_TURN, twist);
                }
            }
            targetShooterRPM = rpmForDistance(tagRange);
        }

        if (gamepad1.left_trigger > 0.1 || aiming) {
            shooterSetRPM = targetShooterRPM;
        }
        else {
            shooterSetRPM = 0;
        }

        if (isReversed) {
            drive = -drive;
            strafe = -strafe;
        }

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist),

        };

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        Rightfront.setPower(speeds[0]);
        Rightback.setPower(speeds[1]);
        Leftfront.setPower(speeds[2]);
        Leftback.setPower(speeds[3]);

        Intake.setPower(intake_power);
        Feeder.setPower(feeder_power);
        Shooter.setVelocity(shooterSetRPM*28.0/60.0);
        double ShooterAmps = Shooter.getCurrent(CurrentUnit.AMPS);
        currentShooterRPM = Shooter.getVelocity()*60.0/28.0;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Orientation", isReversed ? "REVERSED (Back is Front)" : "NORMAL");
        telemetry.addData("Alliance (Y to toggle)", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Tag visible", tag != null);
        telemetry.addData("Auto aim (hold X)", aiming ? "ACTIVE" : "off");
        telemetry.addData("Tag bearing", "%4.1f", tagBearing);
        telemetry.addData("Tag range", "%4.1f", tagRange);
        telemetry.addData("Target RPM", targetShooterRPM);
        telemetry.addData("Current RPM", currentShooterRPM);
        telemetry.addData("Shooter Amps","%4.2f", ShooterAmps);
        telemetry.addData("Intake Power","%4.2f",intake_power);
        telemetry.addData("Feeder Power","%4.2f",feeder_power);
        telemetry.addData("IntakeBar Position","%4.2f", intakebar.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }

    private AprilTagDetection findTag(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
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