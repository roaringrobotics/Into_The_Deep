package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

@Autonomous
public class help extends LinearOpMode {
    //Hardware and variables
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor blackHook;
    DcMotor blueHook;
    DcMotor blueLift;
    public DcMotor blackLift;
    public Servo blackExtend;
    public Servo blueExtend;
    public Servo blackGrip;
    public Servo blueGrip;
    DcMotor inTake;
    Servo blueBar;
    Servo blackBar;
    Servo outtakeFlip;
    Servo blueFlap;
    Servo blackFlap;
    Servo blackIntake;
    Servo blueIntake;
    Servo Liftservo;
    ElapsedTime run = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //All movement happen
        Hardware hw = new Hardware(hardwareMap);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        blueLift = hardwareMap.dcMotor.get("blueLift");
        blackLift = hardwareMap.dcMotor.get("blackLift");
        blackExtend = hardwareMap.servo.get("blackExtend");
        blueExtend = hardwareMap.servo.get("blueExtend");
        blackGrip = hardwareMap.servo.get("blackGrip");
        blueGrip = hardwareMap.servo.get("blueGrip");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime run = new ElapsedTime();


        waitForStart();

        // Resets the timer upon starting
        run.reset();

        // While loop that moves to the left
        //strihgt
        while (run.milliseconds() < 700 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

        } run.reset();
        //starfe right
        while (run.milliseconds() < 945 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        } run.reset();
        //rotate
        while (run.milliseconds() < 720 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        } run.reset();
        //
        while (run.milliseconds() < 1300 && !isStopRequested()){
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        } run.reset();
        while (run.milliseconds() < 270 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        } run.reset();
        while (run.milliseconds() <1800 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }run.reset();

        hw.blueLift.setTargetPosition(-3250);
        hw.blackLift.setTargetPosition(-3250);

        hw.blackLift.setPower(.3);
        hw.blueLift.setPower(.3);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Wait until there
        while (hw.blackLift.getCurrentPosition() > -3250 && !isStopRequested()) {
            sleep(1);
        }

        hw.blueLift.setTargetPosition(0);
        hw.blackLift.setTargetPosition(0);

        hw.blackLift.setPower(.3);
        hw.blueLift.setPower(.3);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hw.blackLift.getCurrentPosition() < 0 && !isStopRequested()) {
            sleep(5500);
        } run.reset();
        while (run.milliseconds() <1800 && !isStopRequested()){
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        }run.reset();
        while (run.milliseconds() <450 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        }run.reset();
        while (run.milliseconds() <1800 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }run.reset();
        while (run.milliseconds() <1800 && !isStopRequested()){
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        }run.reset();
        while (run.milliseconds() <300 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        }run.reset();
        while (run.milliseconds() <1800 && !isStopRequested()){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }run.reset();


    }

}
