package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="8740 Teleop", group="Iterative OpMode")

public class Drive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Leftfront = null;
    private DcMotor Rightfront = null;
    private DcMotor Leftback = null;
    private DcMotor Rightback = null;
    private DcMotor Intake = null;
    private DcMotor Feeder = null;
    private Servo intakebar = null;
    // Track if the robot is currently inverted/reversed
    boolean isReversed = false;

    // Track whether the 'A' button was pressed on the previous loop cycle
    boolean lastGamepad1A = false;

    private DcMotor Shooter = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        Leftback = hardwareMap.get(DcMotor.class, "Leftback");
        Rightback = hardwareMap.get(DcMotor.class, "Rightback");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Feeder = hardwareMap.get(DcMotor.class, "feeder");
        intakebar = hardwareMap.get(Servo.class, "intakebar");
        //Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        //R,R,R,F
        Leftfront.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.REVERSE);
        Leftback.setDirection(DcMotor.Direction.FORWARD);
        Rightback.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Feeder.setDirection(DcMotor.Direction.FORWARD);
       //Shooter.setDirection(DcMotor.Direction.FORWARD);

        Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        //
        if (gamepad1.x) {
            intakebar.setPosition(0.25);
            intakebar.close();
            //PwmControl pwmCtrl = (PwmControl) intakebar;
            //pwmCtrl.setPwmDisable();
        }
        if (gamepad1.b) {
            intakebar.setPosition(0.75);
            intakebar.close();
            //PwmControl pwmCtrl = (PwmControl) intakebar;
            //pwmCtrl.setPwmDisable();
        }

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        // 1. Check for a single, clean tap on the A button (Rising Edge Detection)
        if (gamepad1.a && !lastGamepad1A) {
            isReversed = !isReversed; // Toggle back and forth between true and false
        }
        lastGamepad1A = gamepad1.a; // Save current button state for the next loop iteration

        double drive = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double twist = gamepad1.right_stick_x;
        double intake_power = -gamepad1.right_trigger;
        double feeder_power = gamepad1.right_bumper ? 1.0 : 0.0; // Converted boolean to double power

        if (isReversed) {
            drive = -drive;
            strafe = -strafe;
        }

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist),
                intake_power,
                feeder_power,
                //Shooter_power
        };

        // Send calculated power to wheels
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
            Intake.setPower(speeds[4]);
            Feeder.setPower(speeds[5]);
            //Shooter.setPower(speeds[6]);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Orientation", isReversed ? "REVERSED (Back is Front)" : "NORMAL");
        telemetry.update();

        }
    }