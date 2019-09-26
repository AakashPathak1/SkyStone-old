package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Two Wheels")
public class TwoWheels extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        boolean slowMode = false;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }
            double drive = -gamepad1.left_stick_x;
            double turn = gamepad1.left_stick_y;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);
            if (slowMode) {
                left.setPower(-leftPower / 2);
                right.setPower(-rightPower / 2);
            } else {
                left.setPower(-leftPower);
                right.setPower(-rightPower);
            }

            telemetry.addData("Power: ", leftPower);
            telemetry.update();
        }
    }
}
