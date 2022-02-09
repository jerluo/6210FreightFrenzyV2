package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.ArrayList;
import java.util.Collections;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;

public class VuforiaBM {

    private LinearOpMode robot;
    private VuforiaLocalizer vuforia;

    public VuforiaBM(LinearOpMode robot) {
        this.robot = robot;
        //Create vuforia object
        this.vuforia = null;

        //Add initialization
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        //LogitechC310 = hardwareMap.get(WebcamName.class, "Logitech C310");

        //localizer for webcam
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQt2xVL/////AAABmXIVKUnTcEJbqvVBjp/Sw/9SqarohYyKotzRjT/Xl1/S8KDwsFHv/zYw6rXqXTjKrnjk92GfBA4hbZaQP17d1N6BiBuXO2W/hFNoMGxiF+fWlnvtDmUM1H/MF9faMOjZcPNjnQ7X8DVwdDDha3A3aqaoegefkKxb4A5EjP8Xcb0EPJ1JA4RwhUOutLbCDJNKUq6nCi+cvPqShvlYTvXoROcOGWSIrPxMEiOHemCyuny7tJHUyEg2FTd2upiQygKAeD+LN3P3cT02aK6AJbQ0DlQccxAtoo1+b//H6/eGro2s0fjxA2dH3AaoHB7qkb2K0Vl7ReFEwX7wmqJleamNUG+OZu7K3Zm68mPudzNuhAWQ";
        //paramWC.cameraName = LogitechC310;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        com.vuforia.Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
    }

    public Bitmap getBitmap() throws InterruptedException {
        Bitmap bm = null;
        Image rgb = null;

        // Grab frame
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        // Iterate through all the images
        long num = frame.getNumImages();
        for(int i = 0; i < num; i++){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                rgb = frame.getImage(i);
            }
        }

        // Create bitmap
        bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;

    }

    //returns 1, 2, or 3 for the three possible capstone locations
    // 1 : left || 2 : mid || 3 : right
    // Resolution : 1280 X 720
    // RED CLOSE TO CAROUSEL COORDINATES : RIGHT (3) - 698, 600 || MID (2) - 232, 600
    // BLUE CLOSE TO CAROUSEL COORDINATES : LEFT (1) - || MID (2) -
    public int redCarouselPosition() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        int p3 = blue(bm.getPixel(1140, 640));
        int p2 = blue(bm.getPixel(581, 640));
        int p1 = blue(bm.getPixel(30, 640));

        //threshold values for capstone color
        if (p3 < 50) return 3;
        if (p2 < 50) return 2;
        if (p1 < 50) return 1;
        else return 1;

    }

    public int redWarehousePosition() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        int p1 = blue(bm.getPixel(700, 640));
        int p2 = blue(bm.getPixel(1260, 640));

        //threshold values for capstone color
        if (p1 < 50) return 1;
        if (p2 < 50) return 2;
        else return 3;

    }

    public int blueCarouselPosition() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        int p1 = blue(bm.getPixel(407, 450));
        int p2 = blue(bm.getPixel(930, 450));

        //Threshold values for capstone color
        if (p1 < 65) return 1;
        if (p2 < 65) return 2;
        else return 3;

    }

    public int blueWarehousePosition() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        int p2 = blue(bm.getPixel(290, 450));
        int p3 = blue(bm.getPixel(856, 450));

        //threshold values for capstone color
        if (p2 < 55) return 2;
        if (p3 < 55) return 3;

        else return 1;

    }




}
