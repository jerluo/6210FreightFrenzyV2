package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

public class FreightFrenzyLinearOpMode extends LinearOpMode {

    //-----DECLARE HARDWARE COMPONENTS-----//

    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;
    public DcMotor intake;
    public DcMotor shooter;
    public DcMotor wobbleArm;
    public Servo wobbleClaw;
    public Servo loader;

    //-----DECLARE IMU VARIABLES-----//

    public BNO055IMU imu;
    Orientation angles;

    //-----DEFINE WHEEL/MOTOR SPECIFICATIONS-----//

    static final double     COUNTS_PER_MOTOR_REV    = 560 ; // REV Motor Encoder (1120 for 40:1) (560 for 20:1) (336 for 12:1)
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;   // This is < 1.0 if geared UP (ratio is 2:1)
    static final double     WHEEL_DIAMETER_INCHES   = 4;    // For figuring out circumference

    private double encoderToInches = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);
    private double strafeEncoderToInches = 60;

    //-----OPENCV VARIABLES-----//

    OpenCvCamera logitech_webcam;
    CapstonePipeline pipeline;
    boolean leftPos;

    double minPower = 0.3;

    //-----TELE-OP TOGGLE VARIABLES-----//
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();

    /**
     * NOTE: It is important to make sure that no matter which method is running, if you hit the STOP button on the DS, it shouldn't throw an error.
     *       If you try this and see that the method is throwing an error, add "throws Interruptedexception" in this part of the method declaration:
     *       public void methodName() throws Interruptedexception {}
     */

    //-----INITIALIZATION METHODS-----//

    /**
     * PURPOSE: Initializes hardware components and IMU (gyro)
     * @param map - always write 'hardwareMap' here
     * @param auto - write...
     *              0 if program is NOT auto,
     *             -1 if robot starts on left,
     *              1 if robot starts on right
     * NOTE: IMU will only initialize if auto is NOT 0
     */
    public void init(HardwareMap map, int auto){

        LF          = map.dcMotor.get("LF");
        RF          = map.dcMotor.get("RF");
        LB          = map.dcMotor.get("LB");
        RB          = map.dcMotor.get("RB");
        intake      = map.dcMotor.get("intake");
        shooter     = map.dcMotor.get("shooter");
        wobbleArm   = map.dcMotor.get("wobbleArm");
        wobbleClaw  = map.servo.get("wobbleClaw");
        loader      = map.servo.get("loader");
        imu         = map.get(BNO055IMU.class, "imu"); // Check which IMU is being used

        LF.setDirection(DcMotorSimple.Direction.REVERSE); // Goes backward on positive speed so reverse needed
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE); // Goes backward on positive speed so reverse needed
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleArm.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up gyro
        angles = new Orientation();

        resetEncoders();

        // Make sure loader is retracted


        if (auto != 0) {
            setLoader(false);
            setWobbleClaw(true);
            BNO055IMU.Parameters bparameters = new BNO055IMU.Parameters();
            bparameters.mode = BNO055IMU.SensorMode.IMU;
            bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            bparameters.loggingEnabled = false;

            imu.initialize(bparameters);

            angles = imu.getAngularOrientation(); // Get initial angular orientation

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            // if auto value is -1, set left position variable to true
            leftPos = auto == -1 ? true : false;
            telemetry.addData("IMU calib status", imu.getCalibrationStatus().toString());
            telemetry.addData("Arm angle: ", wobbleArm.getCurrentPosition());
            telemetry.update();
        }

        // Final message
        telemetry.addData("Status: ", "All Initialized");
        telemetry.update();
    }

    /** PURPOSE: Initializes org.firstinspires.ftc.teamcode.OpenCV variables
     *  Note: In here, you can set the camera direction and orientation
     */
    public void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        logitech_webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Logitech C310"), cameraMonitorViewId);
        pipeline = new CapstonePipeline(leftPos);
        logitech_webcam.setPipeline(pipeline);

        //logitech_webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        // ^ not supported with camera type

        logitech_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                logitech_webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT); // change to upright phone orientation
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

    //-----UTILITY METHODS-----//

    /**
     * PURPOSE: HashMap with booleans to make sure toggle buttons actually toggle
     * @param name: unique string for button
     * @param button: boolean button from gamepad
     * @return boolean to toggle on or off
     */

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)){
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    /**
     * PURPOSE: Reset all drivetrain encoders
     */
    public void resetEncoders(){
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
    }

    /**
     * PURPOSE: Get average of drivetrain encoders
     * NOTE: Only takes the average of the front two motors bc they seem most accurate; the back two motors are off - one is a bit over, one is a lot under
     */
    public int getEncoderAvg(){
        /*int avg = (Math.abs(LF.getCurrentPosition())
                + Math.abs(LB.getCurrentPosition())
                + Math.abs(RB.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()))/4;*/

        int avg = (Math.abs(LF.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()))/2;

        return avg;
    }

    /**
     * PURPOSE: Set powers to the drivetrain motors in varying formats
     * NOTE: More detailed instructions written in comments above cases
     * @param type - set the type of format you want, the options are in the case switch within the method
     * @param lf_L - power for either only LF motor, all left motors, or all motors (depends on type specified)
     * @param rf_R - power for either only RF motor or for all right motors (depends on type specified)
     * @param lb - power for LB motor
     * @param rb - power for RB motor
     */
    public void setMotorPowers(String type, double lf_L, double rf_R, double lb, double rb) {
        double[] powers = {lf_L,rf_R,lb,rb};

        for (int i = 0; i < 4; i++){
            if (powers[i] != 0)
                powers[i] = (powers[i] > 0) ? Range.clip(powers[i],minPower,1) : Range.clip(powers[i],-1,-minPower);
        }

        switch (type) {

            // ALL - all motor powers will be set to the value in the first double parameter
            case "ALL":
                LF.setPower(powers[0]);
                RF.setPower(powers[0]);
                LB.setPower(powers[0]);
                RB.setPower(powers[0]);
                break;

            // SIDES - the left motors will be set to the powers in the first double parameter
            // and the right motors will be set to the powers in the second double parameter
            case "SIDES":
                LF.setPower(powers[0]);
                RF.setPower(powers[1]);
                LB.setPower(powers[0]);
                RB.setPower(powers[1]);
                break;

            // EACH - each motor will be set to the powers in the corresponding parameters in the order lf-rf-lb-rb
            case "EACH":
                LF.setPower(powers[0]);
                RF.setPower(powers[1]);
                LB.setPower(powers[2]);
                RB.setPower(powers[3]);
                break;

            // STRAFE - all motors will strafe at the power given in the first double parameter
            // *** If you want to strafe right, give + power (- power for left)
            case "STRAFE":
                LF.setPower(powers[0]);
                RF.setPower(-powers[0]);
                LB.setPower(-powers[0]);
                RB.setPower(powers[0]);
                break;

            // STRAFE ADJUST - left rotating forces will be set to power given in first double parameter
            // right rotating forces will be set to power given in second double parameter
            case "STRAFE ADJUST":
                LF.setPower(powers[1]);
                RF.setPower(-powers[1]);
                LB.setPower(-powers[0]);
                RB.setPower(powers[0]);
                break;

            default:
                telemetry.addData("Error:", "motor powers set incorrectly");
        }
        telemetry.update();
    }

    /**
     * PURPOSE: Displays telemetry of all motor information, such as power and encoder position
     * NOTE: does NOT include telemetry.update(), so you'll have to add that in your program - this is so that you can add other info if you want before updating telemetry
     */
    public void motorTelemetry(){
        telemetry.addData("avg:", getEncoderAvg());
        telemetry.addData("LF Power", LF.getPower() + " " + LF.getCurrentPosition());
        telemetry.addData("RF Power", RF.getPower() + " " + RF.getCurrentPosition());
        telemetry.addData("LB Power", LB.getPower() + " " + LB.getCurrentPosition());
        telemetry.addData("RB Power", RB.getPower() + " " + RB.getCurrentPosition());
    }

    /**
     * PURPOSE: Stop all drivetrain motors
     */
    public void stopMotors() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    /**
     * PURPOSE: Get IMU angle on a -180 to 180 scale
     * NOTE: Remember that this is on a -180 to 180 scale - NOT 0 to 360. This means that you will figure out the angle you want to turn to on a -180 to 180 scale.
     *       Also, contrary to what you'd think, LEFT TURN = POSITIVE direction and RIGHT TURN = NEGATIVE direction.
     * @return the IMU's yaw angle
     */
    public double get180Yaw() { return imu.getAngularOrientation().firstAngle; }

    //-----VISION METHODS-----//

    /**
     * PURPOSE: Detect the number of rings in the stack
     * TIPS: Make sure the robot is centered on the line and aligned against the wall as accurately as possible.
     * NOTE: Works pretty well, regardless of lighting differences - just make sure surroundings are fairly bright.
     * IMPORTANT: Make sure the GREEN WEBCAM LIGHT is ON before hitting start, or it will return 4 no matter what.
     * @return # of rings in stack
     */
    public int detectStack(){

        sleep(1000);

        int numRings = pipeline.getPosition();
        int avg = pipeline.getAnalysis();

        telemetry.addData("numRings:", numRings);
        telemetry.addData("Analysis: ", avg);
        telemetry.update();
        sleep(1000);
        return numRings;
    }

    /**
     * PURPOSE: Use this method when simply testing the stack detection in a teleop program
     */
    public void detectStackTesting(){
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.getPosition());
        sleep(50);
        telemetry.update();
    }

    //-----DRIVETRAIN METHODS-----//
    /**
     * PURPOSE: Implement holonomic drive (arcade) with mecanum wheels. Left joystick drive, right joystick turning.
     * @param leftY - Left joystick y
     * @param leftX - Left joystick x
     * @param rightX - Right joystick x
     * @return motor powers array [LF, RF, LB, RB]
     */
    public double[] holonomicDrive(double leftY, double leftX, double rightX){
        double[] motorPower = {0.0, 0.0, 0.0, 0.0};

        motorPower[0] = leftY + leftX + rightX;
        motorPower[1] = leftY - leftX - rightX;
        motorPower[2] = leftY - leftX + rightX;
        motorPower[3] = leftY + leftX - rightX;

        return scalePower(motorPower);
    }

    /**
     * PURPOSE: Implement field oriented holonomic drive (arcade) with using trig. Movement is based on cardinal directions (ex: forward is always toward goal no matter orientation)
     * @param leftY - Left joystick y
     * @param leftX - Left joystick x
     * @param rightX - Right joystick x
     * @return motor powers array [LF, RF, LB, RB]
     */
    public double[] fieldOriented(double leftX, double leftY, double rightX, double zeroAng){
        double[] motorPower = new double[4];
        double zAngle = get180Yaw();

        double magnitude = Math.hypot(leftX, leftY); //How fast it goes (slight push is slow etc)

        zAngle -= zeroAng;
        if (zAngle < -180){
            zAngle += 360;
        }

        leftY *= -1;
        leftX *= -1;

        double angle = Math.atan2(leftY, leftX) + Math.toRadians(zAngle) - Math.PI / 2; //Angle the joystick is turned in
        double rotation = rightX;

        motorPower[0] = magnitude * Math.sin(angle - Math.PI / 4) + rotation; //Left front motor
        motorPower[1] = magnitude * Math.sin(angle + Math.PI / 4) - rotation; //Right front motor
        motorPower[2] = magnitude * Math.sin(angle + Math.PI / 4) + rotation; //Left back motor
        motorPower[3] = magnitude * Math.sin(angle - Math.PI / 4) - rotation; //Right back motor

        motorPower = scalePower(motorPower);

        return motorPower;

    }

    /**
     * PURPOSE: Scale powers to the correct proportional ratios when powers are greater than 1
     * @param power - motor powers array [LF, RF, LB, RB]
     * @return corrected motor powers array [LF, RF, LB, RB]
     */
    public double[] scalePower(double[] power){
        //Find max
        double max = Math.abs(power[0]);
        for (int i = 1; i < power.length; i++){
            double p = Math.abs(power[i]);
            if (p > max) max = p;
        }

        if (max > 1){
            //Scale powers from max
            for (int i = 0; i < power.length; i++){
                power[i] /= max;
            }
        }

        return power;
    }

    /**
     * PURPOSE: Set motor powers
     * @param powers - motor powers array [LF, RF, LB, RB]
     * @param halfspeed - set motors at half the power
     */
    public void setEachPower(double[] powers, boolean halfspeed){
        if (halfspeed){
            for (int i = 0; i < powers.length; i++){
                powers[i] = powers[i]/2;
            }
        }

        LF.setPower(Range.clip(powers[0], -1, 1));
        RF.setPower(Range.clip(powers[1], -1, 1));
        LB.setPower(Range.clip(powers[2], -1, 1));
        RB.setPower(Range.clip(powers[3], -1, 1));
    }

    /**
     * PURPOSE: Proportional autoturn to target angle
     * @param zeroAng - 0 to 360 reset angle
     * @param tarAng - target orientation robot should turn to (-180 to 180 angle)
     * @param coefficient - how fast the robot turns proportionally
     * @return motor powers array [LF, RF, LB, RB]
     */
    public double[] autoTurn(double zeroAng, double tarAng, double coefficient){
        double[] power = {0.0, 0.0, 0.0, 0.0};
        double angle = get180Yaw();
        double error, mPower = 0;

        //Change this coefficient if you want the turn to have less waggle but be slower
        //double coefficient = 0.015;

        //Find angle
        angle -= zeroAng;
        if (angle < -180){
            angle += 360;
        }

        error = tarAng - angle;

        //If it needs to turn right
        if (error > 180) {
            error = tarAng - Math.abs(angle);
            error *= -1;
        }
        mPower = error * coefficient;

        power[0] = -mPower;
        power[1] = mPower;
        power[2] = -mPower;
        power[3] = mPower;

        /*telemetry.addData("error", error);
        telemetry.addData("zeroang", zeroAng);
        telemetry.addData("angle", angle);
        telemetry.addData("before angle", get180Yaw());*/

        return scalePower(power);
    }

    /**
     * PURPOSE: Drive forward/backward with no correction
     * NOTE: Without corrections, the driving veers severely, so I recommend not using this method if you don't need to
     * @param power - write the max power you want to drive at (backwards = - power , forwards = + power)
     * @param inches - # of inches you want to move (always positive value)
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     */
    public void driveDistance(double power, double inches, double seconds){
        double total = inches * encoderToInches;
        double remaining, finalPower;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();
        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < inches * encoderToInches && t.seconds() < seconds){
            remaining = total - getEncoderAvg();
            telemetry.addData("remaining:", remaining);
            finalPower = (remaining/total) * power;
            setMotorPowers("ALL",finalPower,0,0,0);
            telemetry.addData("target:", inches*encoderToInches);
        }
        setMotorPowers("ALL", 0,0,0,0);
    }
    /**
     * PURPOSE: Drive forward/backward w/ gyro correction
     * NOTE: Preferably use this method for driving. The higher the speed, the more inaccurate it is.
     *       -- Also,if you want the correction to be more immediate/aggressive, slightly increase the value of m
     *          However, keep in mind that too high of an m-value MAY cause it to move like a snake since it'll constantly be overshooting and correcting.
     * ISSUES: There is about an inch of sideways deviation (but it really just depends on speed and distance)
     * @param power - write the max power you want to drive at (backwards = - power , forwards = + power)
     * @param inches - # of inches you want to move (always positive value)
     * @param tHeading - the angle you want it to align to while driving (relative to the initialization orientation)
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     */
    public void driveAdjust(double power, double inches, double tHeading, double seconds){
        // ORIENTATION -180 TO 180
        // LEFT = +, RIGHT = -

        // positive error = need to turn left = increase right power
        // negative error = need to turn right = increase left power

        double total = (inches) * encoderToInches; // -2 to account for drift
        double remaining, finalPower, error, lp, rp, m = 1.2;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds) {
            remaining = total - getEncoderAvg();

            if(tHeading == 180 && get180Yaw() < 0) tHeading = -tHeading; // if error is negative, uses -180 as the target heading instead of 180
            error = tHeading - get180Yaw();

            if(Math.abs(error) > 180) error = (error < 0) ? 360 + error : 360 - error;

            finalPower = (remaining / total) * power * 1.5;
            if (finalPower != 0) finalPower = (finalPower > 0) ? Range.clip(finalPower,0.3,1) : Range.clip(finalPower,-1,-0.3);

            rp = finalPower;
            lp = finalPower;

            if (power < 0) {
                if (error > 1){
                    lp *= m;
                } else if (error < -1)
                    rp *= m + 0.25; // The RB motor is much less powerful so the right side needs an extra boost in addition to the correction factor (m), hence the + 0.25
            } else {
                if (error > 1) {
                    rp *= m + 0.25;
                }else if (error < -1)
                    lp *= m;
            }

            setMotorPowers("SIDES",lp,rp,0,0);

            telemetry.addData("left power: ", lp)
                    .addData("right power: ", rp)
                    .addData("error", error)
                    .addData("current angle", get180Yaw())
                    .addData("target angle", tHeading);
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * PURPOSE: Drive forward/backward w/ gyro correction while revving up the shooter
     * NOTE: Preferably use this method for driving. The higher the speed, the more inaccurate it is.
     *       -- Also,if you want the correction to be more immediate/aggressive, slightly increase the value of m
     *          However, keep in mind that too high of an m-value MAY cause it to move like a snake since it'll constantly be overshooting and correcting.
     *       -- the difference from driveAdjust is that within the while loops the code uses a time object
     *          to rev up the motor
     * ISSUES: There is about an inch of sideways deviation (but it really just depends on speed and distance)
     * @param power - write the max power you want to drive at (backwards = - power , forwards = + power)
     * @param inches - # of inches you want to move (always positive value)
     * @param tHeading - the angle you want it to align to while driving (relative to the initialization orientation)
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     * @param max - final speed of the motor, the maximum speed it will reach
     */
    public void driveAdjustShooter(double power, double inches, double tHeading, double seconds, double max){
        // ORIENTATION -180 TO 180
        // LEFT = +, RIGHT = -

        // positive error = need to turn left = increase right power
        // negative error = need to turn right = increase left power

        double total = (inches) * encoderToInches; // -2 to account for drift
        double remaining, finalPower, error, lp, rp, m = 1.5;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        double speed = 0;
        double newTime = -300;
        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds) {

            if (newTime + 100 <= time.milliseconds() && speed < max){
                newTime = time.milliseconds();
                speed += max/5;
                shooter.setPower(speed);
            }

            remaining = total - getEncoderAvg();

            if(tHeading == 180 && get180Yaw() < 0) tHeading = -tHeading; // if error is negative, uses -180 as the target heading instead of 180
            error = tHeading - get180Yaw();

            if(Math.abs(error) > 180) error = (error < 0) ? 360 + error : 360 - error;

            finalPower = (remaining / total) * power;
            if (finalPower != 0) finalPower = (finalPower > 0) ? Range.clip(finalPower,0.3,1) : Range.clip(finalPower,-1,-0.3);

            rp = finalPower;
            lp = finalPower;

            if (power < 0) {
                if (error > 1){
                    lp *= m;
                } else if (error < -1)
                    rp *= m + 0.25; // The RB motor is much less powerful so the right side needs an extra boost in addition to the correction factor (m), hence the + 0.25
            } else {
                if (error > 1) {
                    rp *= m + 0.25;
                }else if (error < -1)
                    lp *= m;
            }

            setMotorPowers("SIDES",lp,rp,0,0);

            telemetry.addData("left power: ", lp)
                    .addData("right power: ", rp)
                    .addData("error", error)
                    .addData("current angle", get180Yaw())
                    .addData("target angle", tHeading)
                    .addData("speed", speed);
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * PURPOSE: Strafe a certain distance without correction
     * NOTE: Without corrections, the strafing is very messed up, so I recommend not using this method if you don't need to
     * @param power - write the max power you want to drive at (left = - power , right = + power)
     * @param inches - # of inches you want to move (always positive value)
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     */
    public void strafeDistance(double power, double inches, double seconds){

        // positive power is right strafe and negative power is left strafe
        double total = inches * strafeEncoderToInches;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds){

            setMotorPowers("STRAFE",power,0,0,0);

            telemetry.addData("target:", total);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * PURPOSE: Strafe a certain distance w/ gyro correction
     * NOTE: If you are going to strafe at all, use this method. The lower the speed, the more inaccurate it tends to be.
     * ISSUES: The # of inches traveled isn't always exactly accurate and there is a bit of forward/backward deviation (depends on the speed).
     * @param power - write the max power you want to drive at (left = - power , right = + power)
     * @param inches - # of inches you want to move (always positive value)
     * @param tHeading - the angle you want it to align to while driving (relative to the initialization orientation)
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     */
    public void strafeAdjust(double power, double inches, double tHeading, double seconds){
        // ORIENTATION -180 TO 180
        // LEFT = +, RIGHT = -

        double total = (inches) * strafeEncoderToInches; // -2 to account for drift
        double remaining, finalPower, error, rp, lp, p = 1.4; //was 1.2
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds) {
            remaining = total - getEncoderAvg();
            error = tHeading - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(Math.abs(error) > 180) error = (error < 0) ? 360 + error : 360 - error;

            finalPower = (remaining / total) * power;
            if (finalPower != 0) finalPower = (finalPower > 0) ? Range.clip(finalPower,0.3,1) : Range.clip(finalPower,-1,-0.3);

            rp = finalPower;
            lp = finalPower;

            // when moving right: right rotating forces are LF and RF, left rotating forces are LB and RB
            // when moving left: right rotating forces are LB and RB, left rotating forces are LF and RF

            // positive error = need to turn right
            // negative error = need to turn left

            // note: left strafe is worse than right strafe and may not always reach full distance

            if (power != 0){
                if (power < 0) {
                    if (error > 1)
                        rp *= p;
                    else
                        lp *= p;
                } else {
                    if (error < -1) // why is it the same changes as when error > 1 ??? hmmmm
                        rp *= p;
                    else
                        lp *= p;
                }
            }

            setMotorPowers("STRAFE ADJUST", lp, rp,0,0);

            telemetry.addData("target:", total);
            telemetry.addData("error", error);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * PURPOSE: Rotate using PID control
     * NOTE: You'll have to individually fine tune each PID turn with the kP, kI, and kD constants.
     *   --> I believe the best practice is to start with kP, then add a VERY small value for kI, then add kD if needed.
     *   --> You can also check out this page for more details: https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
     * ISSUES: The minimum speed the motors can turn properly at is 0.3, which might be disadvantage to turning since slower is better - however, you can work around it.
     * EXAMPLES OF ALREADY TUNED PID TURNS:
     *      turnPID(90,0.8/180,0.0001,0.5,5000); <-- you can use these!
     *      turnPID(180,0.8/180,0.00005,0.1,5000); <-- ^
     * @param tAngle - the angle you want it to turn to on a -180 t0 180 scale (relative to the initialization orientation)
     * @param kP - usually something/180
     * @param kI - usually something super tiny like 0.0001 or 0.00001
     * @param kD - usually something from 0-1 like 0.5
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     */
    public void turnPID(double tAngle, double kP, double kI, double kD, double seconds){

        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES
        currTime = 0.0;

        ElapsedTime t = new ElapsedTime(); //CREATE NEW TIME OBJECT
        t.reset();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < seconds){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180) error = (error-360);
            else  if(error < -180) error = (error+360);

            if (Math.abs(error) < 30) minPower = 0.20;

            prevTime = currTime;
            currTime = t.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            //A (-) POWER TURNS LEFT AND A (+) TURNS RIGHT
            setMotorPowers("SIDES",-power, power,0,0);
            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
        minPower = 0.3;
    }

    /**
     * PURPOSE: Rotate using PID control
     * NOTE: You'll have to individually fine tune each PID turn with the kP, kI, and kD constants.
     *   --> I believe the best practice is to start with kP, then add a VERY small value for kI, then add kD if needed.
     *   --> You can also check out this page for more details: https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
     * ISSUES: The minimum speed the motors can turn properly at is 0.3, which might be disadvantage to turning since slower is better - however, you can work around it.
     * EXAMPLES OF ALREADY TUNED PID TURNS:
     *      turnPID(90,0.8/180,0.0001,0.5,5000); <-- you can use these!
     *      turnPID(180,0.8/180,0.00005,0.1,5000); <-- ^
     * @param tAngle - the angle you want it to turn to on a -180 t0 180 scale (relative to the initialization orientation)
     * @param kP - usually something/180
     * @param kI - usually something super tiny like 0.0001 or 0.00001
     * @param kD - usually something from 0-1 like 0.5
     * @param seconds - # of seconds you want to allow it to attempt moving before exiting the method (in case robot gets stuck)
     * @param opposite_turn - write...
     *              false for default turn direction,
     *              true for opposite of default turning direction
     *              NOTE: The default turning direction will always be the shortest route; if you want to take the long way around for whatever reason, set it to true
     */
    public void turnPID_new(double tAngle, double kP, double kI, double kD, double seconds, boolean opposite_turn){

        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES
        currTime = 0.0;

        ElapsedTime t = new ElapsedTime(); //CREATE NEW TIME OBJECT
        t.reset();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < seconds){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if (!opposite_turn) { // GET SHORTER TURN ANGLE
                if(error > 180) error = (error-360);
                else  if(error < -180) error = (error+360);
            }

            if (Math.abs(error) < 30) minPower = 0.20;

            prevTime = currTime;
            currTime = t.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            //A (-) POWER TURNS LEFT AND A (+) TURNS RIGHT
            setMotorPowers("SIDES",-power, power,0,0);
            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("error: ", error)
                    .addData("opposite: ", opposite_turn)
                    .addData("currTime: ", currTime);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
        minPower = 0.3;
    }

    //-----MANIPULATOR MECHANISM METHODS-----//

    /**
     * PURPOSE: Set the loader stick to a certain position
     * NOTE: There is a 1500 millisecond wait time to allow it to complete the servo movement before continuing to the next action - you can reduce it if needed.
     * @param deploy - True = extends stick, False = retracts stick
     */
    public void setLoader(boolean deploy) {
        if (deploy)
            loader.setPosition(0);
        else
            loader.setPosition(1);

        sleep(600);

    }

    /**
     * PURPOSE: Start up the shooter flywheel
     * NOTE: I set the max speed to 0.75, but you can increase/decrease it depending on how high/far you want to shoot.
     *       Also, the brake mode should be a FLOAT for the fly wheel & increment/step up to full power for flywheel (within like half a second).
     * @param max - enter the speed you want to set the flywheel to
     */
    public void startShooter(double max){

        double speed = 0;

        while (speed < max) {
            speed += max/5;
            shooter.setPower(speed);
            telemetry.addData("shooter speed:", speed);
            telemetry.update();
            sleep(300);
        }
    }

    public void ShooterSpeed(double max){

    }

    public void setWobbleArm(boolean deployed) {
        wobbleArm.setPower(1);
        wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (deployed) {
            wobbleArm.setTargetPosition(550);
            sleep(1000);
        }
        else{
            wobbleArm.setTargetPosition(0);
            //sleep(2000);
        }
    }

    public void setWobbleClaw(boolean deployed) {
        if (deployed) {
            wobbleClaw.setPosition(0);
        }
        else {
            wobbleClaw.setPosition(1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}