package org.firstinspires.ftc.teamcode.testPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Color Test", group = "TeleOp")
public class ColorTest extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        Manipulators manip = new Manipulators(hardwareMap);

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Red", manip.getVoltage());
            telemetry.update();
        }

    }
}
