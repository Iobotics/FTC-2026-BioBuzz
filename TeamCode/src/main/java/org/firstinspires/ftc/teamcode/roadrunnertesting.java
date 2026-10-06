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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Disabled
@Config
@Autonomous(name = "roadrunnertesting", group = "Autonomous")
public class roadrunnertesting extends LinearOpMode {



public class intake {
    private DcMotorEx intake;

    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}

public class outake {
    private DcMotorEx outake;

    public outake(HardwareMap hardwareMap) {
        outake = hardwareMap.get(DcMotorEx.class, "outake");
        outake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}


public class ServoController {
    private Servo servo;

    public ServoController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
    }
}


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-61, 18, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
          outake outake = new outake(hardwareMap);
          intake intake = new intake(hardwareMap);

        // vision here that outputs positi on
        int visionOutputPosition = 1;


Action trajectoryAction = drive.actionBuilder(initialPose)

        .strafeTo(new Vector2d(-52, 18))

        .strafeTo(new Vector2d(-52, 30))
        .strafeTo(new Vector2d(30, 59))
        .build();


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

    }
}