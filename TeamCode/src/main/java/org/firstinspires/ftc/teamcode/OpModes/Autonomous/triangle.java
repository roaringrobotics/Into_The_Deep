package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Units;

@Autonomous
public class triangle extends LinearOpMode {
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
        Hardware hw = new Hardware(hardwareMap);
        //All movement happen
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
        while (opModeIsActive() && !isStopRequested()) {
            Triangle(run, hw);

        }
    }

    private void Triangle(ElapsedTime run, Hardware hw) {
        while (run.milliseconds() < 700 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);

        }
        run.reset();
        //starfe right
        while (run.milliseconds() < 945 && !isStopRequested()) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        //rotate
        while (run.milliseconds() < 720 && !isStopRequested()) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        //
        while (run.milliseconds() < 1480 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 325 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2280 && !isStopRequested()) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2280 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 500 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 1900 && !isStopRequested()) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 1900 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 300 && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 1700 && !isStopRequested()) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            BasicTelemetry(hw);
        }
        run.reset();
    }

    private void BasicTelemetry(Hardware hw) {
        TelemetryHelper.UpdateTelemetry(telemetry,
                "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                "Position X", hw.imuPos.getPosX(),
                "Position Y", hw.imuPos.getPosY(),
                "bluePos", hw.blueLift.getCurrentPosition(),
                "blackPos", hw.blackLift.getCurrentPosition(),
                "buGripPos", hw.blueGrip.getPosition(),
                "baGripPos", hw.blackGrip.getPosition(),
                "blueTrgt", hw.blueLift.getTargetPosition(),
                "blackTrgt", hw.blackLift.getTargetPosition()
        );

    }
}
