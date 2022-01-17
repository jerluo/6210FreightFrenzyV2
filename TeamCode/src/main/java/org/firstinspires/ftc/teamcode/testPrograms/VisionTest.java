package org.firstinspires.ftc.teamcode.testPrograms;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.FreightFrenzyLinearOpMode;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.OpenCV;
import org.firstinspires.ftc.teamcode.VuforiaBM;

@Config
@Autonomous(name="Vision Test", group = "auto") // BLUE SIDE
//@Disabled

public class VisionTest extends LinearOpMode {

    public static int x1 = 0;
    public static int y1 = 0;

    public static int x2 = 0;
    public static int y2 = 0;

    public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //OpenCV openCV = new OpenCV(this);
        VuforiaBM vuforia = new VuforiaBM(this);
        Bitmap bm = vuforia.getBitmap();
        Manipulators manip = new Manipulators(hardwareMap);




        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            manip.gateTest(pos);
            //telemetry.addLine("1280 x 720 : 1 - left , 3 - right");
            telemetry.addData("val1", blue(bm.getPixel(x1, y1)));
            telemetry.addData("val2", blue(bm.getPixel(x2, y2)));

            telemetry.addData("pos", vuforia.blueWarehousePosition());
            telemetry.update();
        }
    }
}
