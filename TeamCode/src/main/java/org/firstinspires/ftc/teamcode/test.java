/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="test101", group="Linear OpMode")

public class test extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor horizontalSlide = null;
    private DcMotor claw;
    private Servo arm;
    private Servo armLeft;
    private boolean toggle = false; // Initial toggle state
    private boolean lastAState = false; // Keeps track of the previous state of the A button
    private double ArmPosition = 0.0;
    private double leftArmPosition = 0.0;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private final int SLIDE_MAX = 1000;
    private final int SLIDE_MIN = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        //horizontalslide
        horizontalSlide = hardwareMap.get(DcMotor.class,"horzSlide");
        horizontalSlide.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int horizontalSlideMax = 1100;

        //claw
        claw = hardwareMap.get(DcMotor.class,"claw");
        claw.setDirection(DcMotor.Direction.REVERSE);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //arm
        double servoMax = 0.83;
        double servoMin = 0.2;

        arm = hardwareMap.get(Servo.class,"arm");
        arm.setPosition(servoMin);
        armLeft = hardwareMap.get(Servo.class,"armLeft");
        armLeft.setPosition(1-servoMin);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftSlide =hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;
            boolean currentAState = gamepad1.a;
            int HorizontalSlideposition = horizontalSlide.getCurrentPosition();


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;




            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
            int leftSlidePosition = leftSlide.getCurrentPosition();
            int rightSlidePosition = rightSlide.getCurrentPosition();

            if (slidePower > 0 && leftSlidePosition < SLIDE_MAX && rightSlidePosition < SLIDE_MAX) {
                leftSlide.setPower(slidePower);
                rightSlide.setPower(slidePower);
            } else if (slidePower < 0 && leftSlidePosition > SLIDE_MIN && rightSlidePosition > SLIDE_MIN) {
                leftSlide.setPower(slidePower);
                rightSlide.setPower(slidePower);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (!gamepad1.right_bumper){
                max*=2;
            }

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            if (gamepad1.x){
                horizontalSlide.setTargetPosition(horizontalSlideMax);
                horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlide.setPower(.8);

                claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                claw.setPower(0.6);

            }
            else if (gamepad1.y){
                horizontalSlide.setTargetPosition(0);
                horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlide.setPower(.8);

                ArmPosition = servoMin;
                leftArmPosition = 1-servoMin;
                arm.setPosition(ArmPosition);
                armLeft.setPosition(leftArmPosition);

                claw.setTargetPosition(claw.getCurrentPosition());
                claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw.setPower(.2);
            }

            // Detect a button press (transition from not pressed to pressed)
            if (currentAState && !lastAState) {
                toggle = !toggle; // Flip the toggle state
            }
            lastAState = currentAState;

            // Set the servo position based on the toggle state
            if (toggle) {
                ArmPosition = servoMax;
                leftArmPosition = 1-servoMax;
            } else {
                ArmPosition = servoMin;
                leftArmPosition = 1-servoMin;
            }
            arm.setPosition(ArmPosition);
            armLeft.setPosition(leftArmPosition);




            if (gamepad1.b){

                claw.setTargetPosition(claw.getCurrentPosition()-45);
                claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw.setPower(.2);
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            HorizontalSlideposition = horizontalSlide.getCurrentPosition();


            telemetry.addData("Left Slide Position", leftSlidePosition);
            telemetry.addData("Right Slide Position", rightSlidePosition);
            telemetry.addData("Horizontal Slide Position", HorizontalSlideposition);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
