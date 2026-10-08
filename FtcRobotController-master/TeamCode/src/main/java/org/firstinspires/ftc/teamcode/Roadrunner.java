package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
            private ElapsedTime timer = new ElapsedTime();
            private double runTimeSeconds;

            public RunIntake(double runTimeSeconds) {
                this.runTimeSeconds = runTimeSeconds;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(-0.8);
                    timer.reset(); // CRITICAL FIX: Resets the timer right when the action triggers!
                    initialized = true;
                }

                double pos = intakeMotor.getCurrentPosition();
                packet.put("intake", pos);

                if (timer.seconds() < runTimeSeconds) {
                    return true;  // Keep running the action
                } else {
                    intakeMotor.setPower(0);
                    return false; // Action is finished
                }
            }
        }

        // Updated default helper method to run for 4 seconds (instead of 1)
        // to give the robot time to reach (-61, 61)
        public Action runIntake() {
            return new RunIntake(4.0);
        }
    }

/*
    // Nested custom subsystem for Outtake
    public static class Outtake {
        private DcMotorEx outtakeMotor; // Fixed variable casing

        public Outtake(HardwareMap hardwareMap) {
            outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake"); // Fixed variable casing
            outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class RunOuttake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeMotor.setPower(-0.8);
                    initialized = true;
                }

                double pos = outtakeMotor.getCurrentPosition();
                packet.put("outtake", pos); // Fixed telemetry key string mapping

                if (pos > 100.0) {
                    outtakeMotor.setPower(0);
                    return false; // Action is finished
                } else {
                    return true; // Keep running the action
                }
            }
        }

        public Action runOuttake() {
            return new RunOuttake();
        }
    }
*/

    @Override
    public void runOpMode() {
        // Initialize RoadRunner Drive and Subsystems
        Pose2d initialPose = new Pose2d(-47, 61, Math.toRadians(-1.57));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        //Outtake outtake = new Outtake(hardwareMap);

        // Vision setup placeholder
        int visionOutputPosition = 1;

        // Build Trajectories and convert builder into an executable Action
        Action trajectoryAction = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-47, 11.5))
                .waitSeconds(1)
                //.stopAndAdd(outtake.runOuttake())
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-61, 11.5))
                // Starts the intake action immediately when moving toward (-61, 61)
                .afterDisp(0, intake.runIntake())
                .strafeTo(new Vector2d(-61, 61))
                .build();

        // Wait for the driver to press PLAY
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run your autonomous routines concurrently / sequentially here
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction
                )
        );
    }
}
