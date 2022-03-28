package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    public BNO055IMU imu;
    private boolean halfspeed = false;

    public Robot(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        FR = robot.dcMotor.get("rightFront");
        FL = robot.dcMotor.get("leftFront");
        BR = robot.dcMotor.get("rightRear");
        BL = robot.dcMotor.get("leftRear");
        imu = robot.get(BNO055IMU.class, "imu");

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters bparameters = new BNO055IMU.Parameters();
        bparameters.mode = BNO055IMU.SensorMode.IMU;
        bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        bparameters.loggingEnabled = false;

        imu.initialize(bparameters);
    }

    public double[] holonomicPower(double leftX, double leftY, double rightX) {
        double[] motorPower = new double[4];

        motorPower[0] = leftY + leftX + rightX;
        motorPower[1] = leftY - leftX - rightX;
        motorPower[2] = leftY - leftX + rightX;
        motorPower[3] = leftY + leftX - rightX;

        return motorPower;
    }

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

        double angle = Math.atan2(leftY, leftX) + Math.toRadians(zAngle) + Math.PI; //Angle the joystick is turned in
        double rotation = -rightX;

        motorPower[0] = magnitude * Math.sin(angle + Math.PI / 4) - rotation; //Right front motor
        motorPower[1] = magnitude * Math.sin(angle - Math.PI / 4) + rotation; //Left front motor
        motorPower[2] = magnitude * Math.sin(angle - Math.PI / 4) - rotation; //Right back motor
        motorPower[3] = magnitude * Math.sin(angle + Math.PI / 4) + rotation; //Left back motor

        motorPower = scalePower(motorPower);

        return motorPower;

    }

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

    public void setPowers(double[] motorPower) {

        if (halfspeed) {

            for (int i = 0; i<4; i++) {
                motorPower[i] /= 2;
            }

        }

        FR.setPower(motorPower[0]);
        FL.setPower(motorPower[1]);
        BR.setPower(motorPower[2]);
        BL.setPower(motorPower[3]);

    }

    public double get180Yaw() { return imu.getAngularOrientation().firstAngle; }

    public void setHalfspeed() {
        halfspeed = !halfspeed;
    }

    public void pushHalfspeed(boolean down) {
        halfspeed = down;
    }
}
