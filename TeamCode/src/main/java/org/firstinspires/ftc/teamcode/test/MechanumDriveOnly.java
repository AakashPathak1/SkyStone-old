package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp(name = "Mechanum Drive Only")
public class MechanumDriveOnly extends TestChassisOpMode {

    private boolean slowMode = false, accelerating = false;
    private double acceleratePower = 0.0;
    private double v1, v2, v3, v4;

    @Override
    public void runOpMode() {
        initialize();
        boolean lastAState = false;
        composeTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
            v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
            v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
            v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);


            if (gamepad1.y) {
                accelerating = false;
                acceleratePower = 0.0;
            }

            if (slowMode && !accelerating) {
                leftFront.setPower(v1 / 2);
                rightFront.setPower(v2 / 2);
                leftRear.setPower(v3 / 2);
                rightRear.setPower(v4 / 2);
            } else if (!accelerating) {
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }

            if (gamepad1.x) {
                accelerating = true;
                if (acceleratePower < 0.5) {
                    acceleratePower += 0.001;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (acceleratePower > 0 && !gamepad1.x) {
                acceleratePower -= 0.001;
                if (acceleratePower <= 0) {
                    acceleratePower = 0;
                    accelerating = false;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (gamepad1.a && !lastAState) {
                slowMode = !slowMode;
            }
            lastAState = gamepad1.a;

            telemetry.update();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("v1", () -> round(v1, 4));
        telemetry.addLine().addData("v2", () -> round(v2,4));
        telemetry.addLine().addData("v3", () -> round(v3,4));
        telemetry.addLine().addData("v4", () -> round(v4, 4));
        telemetry.addLine().addData("Accelerate Power", () -> round(acceleratePower, 5));
    }

    private static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
