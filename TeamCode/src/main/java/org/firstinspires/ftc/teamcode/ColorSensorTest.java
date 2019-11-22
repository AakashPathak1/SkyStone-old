package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name = "Color Sensor Test", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");

        waitForStart();
        composeTelemetry();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("Alpha1 ", () -> colorSensor.alpha());
        telemetry.addLine().addData("Red1 ", () -> colorSensor.red());
        telemetry.addLine().addData("Green1 ", () -> colorSensor.green());
        telemetry.addLine().addData("Blue1 ", () -> colorSensor.blue());
        telemetry.addLine().addData("Shaunak value1 ", () -> 1.0 * (colorSensor.red() + colorSensor.green()) / (colorSensor.red() + colorSensor.green() + colorSensor.blue()));

        telemetry.addLine().addData("Alpha2 ", () -> colorSensor2.alpha());
        telemetry.addLine().addData("Red2 ", () -> colorSensor2.red());
        telemetry.addLine().addData("Green2 ", () -> colorSensor2.green());
        telemetry.addLine().addData("Blue2 ", () -> colorSensor2.blue());
        telemetry.addLine().addData("Shaunak value2 ", () -> 1.0 * (colorSensor2.red() + colorSensor2.green()) / (colorSensor2.red() + colorSensor2.green() + colorSensor2.blue()));

        telemetry.addLine();

        telemetry.addLine().addData("Difference ", () ->  (Math.abs((1.0 * colorSensor.red() + colorSensor.green()) / ( 1.0 * colorSensor.red() + colorSensor.green() + colorSensor.blue()) - (((double) (colorSensor2.red() + colorSensor2.green()))) / (colorSensor2.red() + colorSensor2.green() + colorSensor2.blue()))));

        telemetry.update();
    }
}
