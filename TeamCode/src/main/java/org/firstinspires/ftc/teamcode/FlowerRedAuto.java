package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;




@Autonomous(name="FlowerRedAuto", group="Linear Opmode")
public class FlowerRedAuto extends LinearOpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left_motor  = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_left_motor   = null;
    private DcMotor back_right_motor  = null;

    private DcMotor shooter_motor = null;

    private DcMotor intake_motor = null;

    @Override
    public void runOpMode() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left_motor   = hardwareMap.get(DcMotor.class, "FL");
        front_right_motor  = hardwareMap.get(DcMotor.class, "FR");
        back_left_motor    = hardwareMap.get(DcMotor.class, "BL");
        back_right_motor   = hardwareMap.get(DcMotor.class, "BR");
        intake_motor       = hardwareMap.get(DcMotor.class, "intake");
        shooter_motor      = hardwareMap.get(DcMotor.class, "shooter");

        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);
        shooter_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }}