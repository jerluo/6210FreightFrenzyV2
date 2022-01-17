package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapstonePipeline extends OpenCvPipeline {

    public boolean leftPosition;

    /**
     * PURPOSE: Create enum class to define the ring position and assign in values for each enum element
     */
    public enum RingPosition
    {
        FOUR(4), ONE(1), NONE(0);

        private final int num;

        /**
         * PURPOSE: Class constructor to initialize int value for each element
         * @param num
         */
        RingPosition(int num){
            this.num = num;
        }

        /**
         * PURPOSE: Get method for each enum element's int value
         * @return
         */
        public int getNum(){
            return num;
        }
    }

    /**
     * PURPOSE: Declare color constants
     */

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /**
     * PURPOSE: Indicate size of region you want to analyze
     */

    static Point REGION1_TOPLEFT_ANCHOR_POINT; // Y IS VERTICAL AND X IS HORIZONTAL ON UPRIGHT FRAME : (0,0) IS TOP LEFT CORNER

    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 40;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 130;  //130

    Point region1_pointA;
    Point region1_pointB;

    /**
     * PURPOSE: Declare working variables
     */

    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile CapstonePipeline.RingPosition position = CapstonePipeline.RingPosition.FOUR;

    /**
     * PURPOSE: Class constructor to set starting position
     * @param lp
     */
    public CapstonePipeline(boolean lp){
        leftPosition = lp;
    }

    /**
     * PURPOSE: This function takes the RGB frame, converts to YCrCb and extracts the Cb channel to the 'Cb' variable
     * @param input
     */
    void inputToCb(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    /**
     * PURPOSE: Depending on robot starting position, set region top left anchor point accordingly
     */
    public void setRegion1TopleftAnchorPoint(){
        if (leftPosition){
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(260,200);
        }else{
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(15,195);
        }

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    }

    @Override
    public void init(Mat firstFrame){

        inputToCb(firstFrame);
        setRegion1TopleftAnchorPoint();
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input){
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        position = CapstonePipeline.RingPosition.FOUR; // Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            position = CapstonePipeline.RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            position = CapstonePipeline.RingPosition.ONE;
        }else{
            position = CapstonePipeline.RingPosition.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis()
    {
        return avg1;
    }

    public int getPosition() {
        return position.getNum();
    }
}
