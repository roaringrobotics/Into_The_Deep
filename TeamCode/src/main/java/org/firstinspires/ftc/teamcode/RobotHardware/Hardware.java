package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Initializes and tracks the Robot Hardware.
public class Hardware {

    public Hardware(HardwareMap hardwareMap) throws InterruptedException {
        InitDriveMotors(hardwareMap);
        InitLiftMotors(hardwareMap);
        InitExtendServos(hardwareMap);
        InitGripServos(hardwareMap);
        InitImu(hardwareMap);
    }

    private void InitImu(HardwareMap hardwareMap) {
        // Initialize IMU and Encoders.  imuPos can be internal IMU/Encoders or Pinpoint computer.
        // imuPos = new ImuImp(hardwareMap);
        imuPos = new PinpointImpl(hardwareMap); // use pinpoint
    }

    private void InitGripServos(HardwareMap hardwareMap) {
        // Grip servos
        // Names need to match configuration on driver hub
        blackGrip = hardwareMap.servo.get("blackGrip");
        blueGrip = hardwareMap.servo.get("blueGrip");
    }

    private void InitExtendServos(HardwareMap hardwareMap) {
        // Extender servos
        // Names need to match configuration on driver hub
        blackExtend = hardwareMap.servo.get("blackExtend");
        blueExtend = hardwareMap.servo.get("blueExtend");
    }

    private void InitLiftMotors(HardwareMap hardwareMap) {
        // Lift motors
        // Names need to match configuration on driver hub
        blueLift = hardwareMap.dcMotor.get("blueLift");
        blackLift = hardwareMap.dcMotor.get("blackLift");

        // Set mode and reset encoder.
//        blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse direction of blue lift.
        blueLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake on zero power
        blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blueLift.setTargetPosition(0);
        blackLift.setTargetPosition(0);

        blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        blueLift.setTargetPosition(0);
        blackLift.setTargetPosition(0);
    }

    private void InitDriveMotors(HardwareMap hardwareMap) {
        // Drive Motors
        // Names need to match configuration on driver hub
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor blueLift;
    public DcMotor blackLift;
    public Servo blackExtend;
    public Servo blueExtend;
    public Servo blackGrip;
    public Servo blueGrip;

    // ImuWrapper is implemented for IMU or Pinpoint.
    public ImuPositionWrapper imuPos;


}