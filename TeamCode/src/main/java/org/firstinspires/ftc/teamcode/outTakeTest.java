package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "outTakeTest", group = "Linear OpMode")
public class outTakeTest extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor outTakeMotor = null;
    private boolean Atoggle = false;
    private boolean lastAState = false;
    private final int Out_Max = 1000; // Set appropriate max value
    private final int Out_Min = 0;    // Set appropriate min value

    @Override
    public void runOpMode() {
        outTakeMotor = hardwareMap.get(DcMotor.class, "outTakeMotor");
        outTakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outTakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outTakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                outTakeMotor.setPower(1.0);
            } else {
                outTakeMotor.setTargetPosition(Out_Min);
                outTakeMotor.setPower(1.0);
            }

            telemetry.addData("Motor Position", outTakeMotor.getCurrentPosition());
            telemetry.addData("Toggle State", Atoggle);
            telemetry.update();
        }
    }
}
