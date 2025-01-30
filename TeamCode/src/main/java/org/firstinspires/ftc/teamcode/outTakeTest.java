package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "outTakeTest", group = "Linear OpMode")
public class outTakeTest extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor outTakeMotor = null;
    private boolean Atoggle = false;
    private boolean servotoggle = false;
    private boolean lastServoState = false;
    private boolean lastAState = false;
    private boolean spinnertoggle = false;
    private boolean lastSpinnerState = false;
    private final int Out_Max = 200; // Set appropriate max value
    private final int Out_Min = 0;    // Set appropriate min value
    private Servo arm = null;
    private Servo armLeft = null;
    private CRServo spinner = null;
    private CRServo spinnerLeft = null;
    private double ArmPosition = 0.0;
    private double leftArmPosition = 0.0;

    @Override
    public void runOpMode() {
        outTakeMotor = hardwareMap.get(DcMotor.class, "outTakeMotor");
        outTakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outTakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outTakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm
        double servoMax = 0.83;
        double servoMin = 0.2;

        arm = hardwareMap.get(Servo.class,"arm");
        arm.setPosition(servoMin);
        armLeft = hardwareMap.get(Servo.class,"armLeft");
        armLeft.setPosition(1-servoMin);

        //spinners
        spinner = hardwareMap.get(CRServo.class,"leftSpin");
        spinnerLeft = hardwareMap.get(CRServo.class,"rightSpin");
        /////-----------------------------------------------------------------------------------

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runTime.reset();

        while (opModeIsActive()) {
            boolean currentAState = gamepad1.a;

            if (currentAState && !lastAState) {
                Atoggle = !Atoggle;
            }
            lastAState = currentAState;

            if (Atoggle) {
                outTakeMotor.setTargetPosition(Out_Max);
                outTakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outTakeMotor.setPower(0.5);
            } else {
                outTakeMotor.setTargetPosition(Out_Min);
                outTakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outTakeMotor.setPower(0.5);
            }

            updateArm();
            updateSpinners();

            telemetry.addData("Motor Position", outTakeMotor.getCurrentPosition());
            telemetry.addData("Toggle State", Atoggle);
            telemetry.update();
        }
    }

    public void updateArm(){
        double servoMax = 0.83;
        double servoMin = 0.2;

        boolean currentServoState = gamepad1.b;
        if (currentServoState && !lastServoState) {
            servotoggle = !servotoggle; // Flip the toggle state
        }
        lastServoState = currentServoState;

        if (servotoggle) {
            ArmPosition = servoMax;
            leftArmPosition = 1-servoMax;
        } else {
            ArmPosition = servoMin;
            leftArmPosition = 1-servoMin;
        }
        arm.setPosition(ArmPosition);
        armLeft.setPosition(leftArmPosition);
    }

    public void updateSpinners(){
        boolean currentSpinnerState = gamepad1.x;
        if (currentSpinnerState && !lastSpinnerState) {
            spinnertoggle = !spinnertoggle; // Flip the toggle state
        }
        lastSpinnerState = currentSpinnerState;

        if (spinnertoggle) {
            spinner.setPower(1.0);
            spinnerLeft.setPower(1.0);
        }
        else {
            spinner.setPower(0.0);
            spinnerLeft.setPower(0.0);
        }
    }
}
