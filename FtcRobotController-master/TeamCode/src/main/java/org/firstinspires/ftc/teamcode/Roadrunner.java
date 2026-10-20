package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RoadRunner", group = "Autonomous")
public class Roadrunner extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Nested custom subsystem for Intake
    public static class Intake {
        private DcMotorEx intakeMotor;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class RunIntake implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;
            private double runTimeSeconds;

            public RunIntake(double runTimeSeconds) {
                this.runTimeSeconds = runTimeSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(0.8);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double pos = intakeMotor.getCurrentPosition();
                packet.put("intake", pos);

                if (timer.seconds() < runTimeSeconds) {
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action runIntake(double seconds) {
            return new RunIntake(seconds);
        }
    }

    // Nested custom subsystem for Feeder
    public static class Feeder {
        private DcMotorEx feederMotor;

        public Feeder(HardwareMap hardwareMap) {
            feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public DcMotorEx getFeederMotor() {
            return feederMotor;
        }

        public class RunFeeder implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;
            private double runTimeSeconds;

            public RunFeeder(double runTimeSeconds) {
                this.runTimeSeconds = runTimeSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feederMotor.setPower(0.8);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double pos = feederMotor.getCurrentPosition();
                packet.put("feeder", pos);

                if (timer.seconds() < runTimeSeconds) {
                    return true;
                } else {
                    feederMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action runFeeder(double seconds) {
            return new RunFeeder(seconds);
        }
    }

    // Nested custom subsystem for Outtake
    public static class Outtake {
        private DcMotorEx outtakeMotor;

        public Outtake(HardwareMap hardwareMap) {
            outtakeMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
            outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class RunOuttake implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeMotor.setPower(0.8);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double pos = outtakeMotor.getCurrentPosition();
                packet.put("Shooter", pos);

                if (timer.seconds() < 1.5) {
                    return true;
                } else {
                    outtakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ShootThenFeed implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;
            private DcMotorEx feederMotor;
            private double spinUpSeconds;
            private double feedSeconds;

            public ShootThenFeed(DcMotorEx feederMotor, double spinUpSeconds, double feedSeconds) {
                this.feederMotor = feederMotor;
                this.spinUpSeconds = spinUpSeconds;
                this.feedSeconds = feedSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeMotor.setPower(0.8);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double pos = outtakeMotor.getCurrentPosition();
                packet.put("Shooter", pos);

                if (timer.seconds() < spinUpSeconds) {
                    // Phase 1: Spin up shooter
                    return true;
                } else if (timer.seconds() < (spinUpSeconds + feedSeconds)) {
                    // Phase 2: Run feeder while shooter stays powered
                    feederMotor.setPower(0.8);
                    packet.put("feeder", feederMotor.getCurrentPosition());
                    return true;
                } else {
                    // Phase 3: Stop both shooter and feeder
                    outtakeMotor.setPower(0);
                    feederMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action runOuttake() {
            return new RunOuttake();
        }

        public Action shootThenFeed(DcMotorEx feederMotor, double spinUpSeconds, double feedSeconds) {
            return new ShootThenFeed(feederMotor, spinUpSeconds, feedSeconds);
        }
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-47, 61, Math.toRadians(-1.57));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        int visionOutputPosition = 1;

        // Build continuous path trajectory
        Action continuousTrajectory = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-47, 11.5), Math.toRadians(0))
                .waitSeconds(1)
                .stopAndAdd(outtake.shootThenFeed(feeder.getFeederMotor(), 0.8, 1.5))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-61, 11.5))

                .afterDisp(0, new ParallelAction(
                        intake.runIntake(4.0),
                        feeder.runFeeder(4.0)
                ))

                .strafeTo(new Vector2d(-61, 61))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(38, 40), Math.toRadians(-90))
                .strafeTo(new Vector2d(38, 61))
                .build();

        // 1. Safe Initialization Loop (Runs when you hit INIT)
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        // 2. Wait for the driver to press the START button
        waitForStart();

        if (isStopRequested()) return;

        // 3. SAFE EXECUTION: Runs the path blocking tracking sequence during the match
        Actions.runBlocking(
                new SequentialAction(
                        continuousTrajectory
                )
        );
    }
}