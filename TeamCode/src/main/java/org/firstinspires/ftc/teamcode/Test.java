package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@Autonomous(name = "Test")
public class Test extends MyOpMode {

    @Override
    public void runOpMode() {
        initialize();
        findSkystoneTest();
    }

    private void findSkystoneTest() {
        colorTelemetry();
        waitForStart();

        sideDrive(0.5, 29, 10);
        sleep(1000);
        encoderDrive(0.2, -20, 10);
        sleep(800);

        leftFront.setPower(-0.2);
        leftRear.setPower(-0.2);
        rightFront.setPower(-0.2);
        rightRear.setPower(-0.2);
        boolean foundSkystone;
        do {
            foundSkystone = 1.0 * (colorSensor.red() + colorSensor.green()) / (colorSensor.red() + colorSensor.green() + colorSensor.blue()) < 0.73;
        } while (!foundSkystone && opModeIsActive());
        brake();
    }

    private void colorTelemetry() {
        telemetry.addLine()
                .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addLine().addData("Alpha ", () -> colorSensor.alpha());
        telemetry.addLine().addData("Red  ", () -> colorSensor.red());
        telemetry.addLine().addData("Green ", () -> colorSensor.green());
        telemetry.addLine().addData("Blue ", () -> colorSensor.blue());
        telemetry.addLine().addData("Shaunak value ", () -> 1.0 * (colorSensor.red() + colorSensor.green()) / (colorSensor.red() + colorSensor.green() + colorSensor.blue()));

        telemetry.update();
    }

    private void gyroTest() {
        gyroTurn(-90);
        telemetry.addData("Angle", angles.firstAngle);
        telemetry.update();
        sleep(500);
        gyroTurn(90);
        sleep(300);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Angle", angles.firstAngle);
        telemetry.update();
        sleep(10000);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
