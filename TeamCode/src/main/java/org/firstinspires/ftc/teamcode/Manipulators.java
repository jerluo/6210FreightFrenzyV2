package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Manipulators {

    private HardwareMap robot = null;

    //Lift
    public DcMotor RL = null;
    // Lowest encoder position
    private int lowest = 0;
    // Highest encoder position
    private int highest = 0;

    // Increase encoder values to move arm lower (further)
    public static int high = 2800;
    public static int mid = 3360;
    public static int low = 3747;

    // Capstone -1967


    //Intake motor
    private DcMotorEx IT;

    //Intake servos
    private DcMotor RC;
    private DcMotor LC;

    private Servo gate;

    ColorSensor color;

    public Manipulators(HardwareMap robot) {
        this.robot = robot;

        IT = robot.get(DcMotorEx.class, "intake");

        RC = robot.get(DcMotor.class, "rightCarousel");
        LC = robot.get(DcMotor.class, "leftCarousel");

        //intake servos
        gate = robot.get(Servo.class, "gate");


        // Lift
        RL = robot.get(DcMotor.class, "rightLift");

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //color sensor
        color = robot.get(ColorSensor.class, "Color");
    }

    public void checkPosition() {
        // Get the bounds of the encoder value of the lift
        if (RL.getCurrentPosition() < lowest) {
            lowest = RL.getCurrentPosition();
        }

        if (RL.getCurrentPosition() > highest) {
            highest = RL.getCurrentPosition();
        }
    }


    // int pos : 0 - return/retract || 1 - low || 2 - mid || 3 - high
    public void automaticLift(int pos)
    {
        int tarPos = 0;
        int height = 0;


        if (pos == 0) height = 0;

        if (pos == 1) height = low;
        if (pos == 2) height = mid;
        if (pos == 3) height = high;


        tarPos = lowest + height;

        // Takes in encoder position to move lift to
        RL.setTargetPosition(tarPos);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RL.setPower(1);
    }

    public void resetArm() {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int liftEncoder() { return RL.getCurrentPosition();}

    public boolean liftIsBusy() {
        return RL.isBusy();
    }

    public void manualLift(double y) {
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setPower(y);
    }
    public void stopLift() {
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setPower(0);
    }

    public void redCarousel(){
        RC.setPower(-0.55);
        LC.setPower(-0.55);
    }

    public void blueCarousel(){
        RC.setPower(0.6);
        LC.setPower(0.6);
    }

    public void initialCarousel(double speed){
        RC.setPower(speed);
        LC.setPower(speed);
    }

    public void Slower(double speed){
        RC.setPower(speed);
        LC.setPower(speed);
    }



    public void teleRedCarousel(boolean fast){
        if (fast){
            RC.setPower(-1);
            LC.setPower(-1);
        }
        else {
            RC.setPower(-0.65);
            LC.setPower(-0.65);
        }
    }

    public void teleBlueCarousel(boolean fast){
        if (fast){
            RC.setPower(1);
            LC.setPower(1);
        }
        else {
            RC.setPower(0.65);
            LC.setPower(0.65);
        }
    }

    public void carouselStop(){
        RC.setPower(0);
        LC.setPower(0);
    }

    public void gate(boolean open) {
        if (open) gate.setPosition(0);
        else gate.setPosition(1);
    }

    public double gatePosition(){
        return gate.getPosition();
    }

    public void gatePos(double pos) {
        if (pos == 3) gate.setPosition(0);
        if (pos == 2) gate.setPosition(0.4);
        if (pos == 1) gate.setPosition(0.5);
    }

    public void gateTest(double pos) {
        gate.setPosition(pos);
    }

    public void intake(boolean out) {
        if (out) IT.setPower(-1);
        else IT.setPower(1);
    }

    public void slowIntake() {
        IT.setPower(0.8);
    }

    public double getVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : robot.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public double intakeVelocity() { return IT.getVelocity();}

    public void intakeStop() {
        IT.setPower(0);
    }

    public boolean senseColor(){

        if (color.red() > 100 || color.green()  > 150) {
            return true;
        }

        return false;
    }

    public int colorValue() {
        return (color.red());
    }

}
