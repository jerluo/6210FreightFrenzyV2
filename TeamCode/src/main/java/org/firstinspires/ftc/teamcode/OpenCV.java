package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class OpenCV {

    private LinearOpMode auto;

    public OpenCV(LinearOpMode auto){
        this.auto = auto;

        int cameraMonitorViewId = auto.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", auto.hardwareMap.appContext.getPackageName());

        WebcamName logitech_webcam = auto.hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(logitech_webcam, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT); // change to upright phone orientation
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void Test() {
        // do stuff...
        auto.telemetry.addData("OpenCV.Test",": hello");
        auto.telemetry.update();
    }
}
