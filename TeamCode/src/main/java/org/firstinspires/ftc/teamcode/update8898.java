package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;




/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="update8898", group="Linear Opmode")
public class update8898 extends OpMode {


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

    private ServoImplEx flowerLeft;
    private ServoImplEx flowerRight;




    private ServoImplEx swiper;


    private double intake_power;


    private double feeder_power;

    double targetOutakeRPM = 2100;




    private boolean isReversed = false;
    private boolean lastA = false;

    private boolean flowerDown = false;


    public double outakeSetRPM = 0.0;
    public double outake2SetRPM = 0.0;




    private ElapsedTime runtime = new ElapsedTime();


    private double servoTimer;


    final double P = 65.0;
    final double I = 1.5;
    final double D = 1.5;
    final double F = 8.0;

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    boolean exposureSet = false;

    final String CAMERA_NAME = "webcam";

    final int EXPOSURE_MS = 6;
    final int GAIN = 250;

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
    final double TURN_SIGN = -1.0;

    boolean blueAlliance = true;
    boolean autoAlignActive = false;
    String alignStatus = "IDLE";
    double autoStartTime = 0;
    double lastTargetTime = 0;
    double alignedSince = -1;
    double searchDir = -1;
    String lockedName = null;
    boolean alliancePrev = false;
    boolean alignBtnPrev = false;

    private static class HiveTarget {
        String name;
        double bearing, range, roll, percent;
    }


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
        flowerLeft = hardwareMap.get(ServoImplEx.class, "flower left");
        flowerRight = hardwareMap.get(ServoImplEx.class, "flower right");






        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        flowerLeft.setDirection(Servo.Direction.REVERSE);
        flowerRight.setDirection(Servo.Direction.FORWARD);


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


        initVision();


        runtime.reset();


    }

    @Override
    public void init_loop() {
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


    @Override
    public void loop() {

        boolean dpad_downPrev = false;
        boolean dpad_upPrev = false;

        flowerLeft.setPwmEnable();
        flowerRight.setPwmEnable();

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

        //Code for raising or lowering the flower arm
        if (gamepad1.xWasPressed()) {
            flowerDown = !flowerDown;
            if(flowerDown) {
                flowerLeft.setPosition(0.34);
                flowerRight.setPosition(0.34);
            }
            if(!flowerDown) {
                flowerLeft.setPosition(0.0);
                flowerRight.setPosition(0.0);
            }
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


        //Code for increase and decreasing speed
        if (gamepad2.dpad_down && !dpad_downPrev) {
            targetOutakeRPM = targetOutakeRPM -50;
        }

        if(gamepad2.dpad_up && !dpad_upPrev) {
            targetOutakeRPM = targetOutakeRPM + 50;
        }




        //Code to run the Outake
        if(gamepad1.left_trigger > 0.0 && gamepad1.right_trigger < 0.01) {
            outakeSetRPM = 2175.0;
            outake2SetRPM = 2175.0;
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


        double now = runtime.seconds();
        setManualExposure();

        if (gamepad1.dpad_right && !alliancePrev) {
            blueAlliance = !blueAlliance;
            lockedName = null;
        }
        alliancePrev = gamepad1.dpad_right;

        List<AprilTagDetection> allTags = aprilTag.getDetections();
        HiveTarget target = findHiveTarget(allTags, autoAlignActive ? lockedName : null);

        boolean alignBtn = gamepad1.b;
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
                lockedName = null;
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
                lockedName = target.name;
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

        if (autoAlignActive) {
            drive = 0;
            strafe = 0;
            twist = autoTwist;
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
        telemetry.addData("Target RPM", targetOutakeRPM);
        telemetry.addData("Outake 1 Amps","%4.2f",outakeAmps);
        telemetry.addData("Outake 2 Amps","%4.2f", outake2Amps);
        telemetry.addData("Servo Position", "%4.2f", servoPosition);
        telemetry.addData("Intake Power", "%4.2f" , intake_power);
        //telemetry.addData("hi lily remember to yell at tom and bella cuz they needa park and garden","");
        //telemetry.addData("this is a little funsies note from carter, the better programmer on 8741", "");
        telemetry.addData("Flower Arm Down?",flowerDown);
        telemetry.addData("Alliance (dpad right)", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Align status (B)", alignStatus);
        if (target != null) {
            telemetry.addData("Target cluster", target.name);
            telemetry.addData("Cluster found", "%5.1f", target.percent);
            telemetry.addData("Bearing (deg)", "%5.1f", target.bearing);
            telemetry.addData("Range (in)", "%5.1f", target.range);
            telemetry.addData("Roll (deg)", "%5.1f", target.roll);
        } else {
            telemetry.addData("Target", "none visible");
        }
        telemetry.addData("Camera state", portal.getCameraState());
        telemetry.addData("# detections (all)", allTags.size());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
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

    private HiveTarget findHiveTarget(List<AprilTagDetection> detections, String preferName) {
        String alliancePrefix = blueAlliance ? "B" : "R";
        HiveTarget best = null;

        for (AprilTagDetection d : detections) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (c.metadata == null || c.ftcPose == null) continue;

            String name = c.metadata.name;
            if (name == null || !name.toUpperCase().startsWith(alliancePrefix)) continue;
            if (Math.abs(c.ftcPose.roll) >= 90) continue;

            HiveTarget t = new HiveTarget();
            t.name = name;
            t.bearing = c.ftcPose.bearing;
            t.range = c.ftcPose.range;
            t.roll = c.ftcPose.roll;
            double pct = c.percentClusterFound;
            t.percent = pct;

            if (preferName != null && preferName.equals(name)) return t;
            if (best == null || t.percent > best.percent) best = t;
        }
        return best;
    }
}
