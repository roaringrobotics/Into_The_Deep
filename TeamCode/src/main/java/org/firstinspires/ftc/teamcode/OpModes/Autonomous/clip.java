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

import java.util.ArrayList;

@Autonomous
public class clip extends LinearOpMode {
    //Hardware and variables
    Hardware hw;

    ElapsedTime run = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //All movement happen
        Hardware hw = new Hardware(hardwareMap);

//        hw.blueExtend.setPosition(0.05);
//        hw.blackExtend.setPosition(0.95);


        hw.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime run = new ElapsedTime();
        double drivePower = 0.60;
        double strafPower = 0.50;









        double[] targetDistance = {
                21.0,
                -0.5,
                30.2,
                -3350,
                -2480,
                -200
        };

        hw.blueGrip.setPosition(hw.closeBlueGrip);
        hw.blackGrip.setPosition(hw.closeBlackGrip);
        sleep(1000);


        waitForStart();

        hw.blueLift.setTargetPosition((int) targetDistance[5]);
        hw.blackLift.setTargetPosition((int) targetDistance[5]);
        hw.blackLift.setPower(.3);
        hw.blueLift.setPower(.3);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Resets the timer upon starting
//        run.reset();
        hw.imuPos.reset();
        hw.imuPos.update();
        double xt = hw.imuPos.getPosX();
        double yt = hw.imuPos.getPosY();
        double hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
        double correction = correction(hDelta);
        BasicTelemetry(hw, xt, yt, correction, drivePower);

        // While loop that moves it straight
        while (hw.imuPos.getPosY() < targetDistance[0] && !isStopRequested()) {
            sleep(1);
            hw.frontLeft.setPower(drivePower + correction);
            hw.frontRight.setPower(drivePower - correction);
            hw.backLeft.setPower(drivePower + correction);
            hw.backRight.setPower(drivePower - correction);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            BasicTelemetry(hw, xt, yt, correction, drivePower);
        }
        run.reset();
        //starfe right

        while (run.milliseconds() < 500 && !isStopRequested()) {
            hw.frontLeft.setPower(-strafPower);
            hw.frontRight.setPower(strafPower);
            hw.backLeft.setPower(strafPower);
            hw.backRight.setPower(-strafPower);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            BasicTelemetry(hw, xt, yt, correction, drivePower);
        }




//        while (hw.imuPos.getPosX() > targetDistance[1] && !isStopRequested()) {
//            sleep(5);
//            hw.frontLeft.setPower(-strafPower);
//            hw.frontRight.setPower(strafPower);
//            hw.backLeft.setPower(strafPower);
//            hw.backRight.setPower(-strafPower);
//            hw.imuPos.update();
//            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
//            correction = correction(hDelta);
//            BasicTelemetry(hw, xt, yt, correction, drivePower);
//        }




        run.reset();
        hw.imuPos.update();
        BasicTelemetry(hw, xt, yt, correction, drivePower);
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);
        //rotate
        hw.blueLift.setTargetPosition((int) targetDistance[3]);
        hw.blackLift.setTargetPosition((int) targetDistance[3]);

        hw.blackLift.setPower(.6);
        hw.blueLift.setPower(.6);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//         Wait until there
        run.reset();
        while (hw.blackLift.getCurrentPosition() > -3130 && !isStopRequested()) {
            BasicTelemetry(hw, xt, yt, correction, drivePower);
            sleep(20);
//            hw.frontLeft.setPower(0.09);
//            hw.frontRight.setPower(0.09);
//            hw.backLeft.setPower(0.09);
//            hw.backRight.setPower(0.09);
        }
        // go towards the rods
        while (hw.imuPos.getPosY() < targetDistance[2] && !isStopRequested()) {
            sleep(1);
            hw.frontLeft.setPower(drivePower + correction);
            hw.frontRight.setPower(drivePower - correction);
            hw.backLeft.setPower(drivePower + correction);
            hw.backRight.setPower(drivePower - correction);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            BasicTelemetry(hw, xt, yt, correction, drivePower);
        }
//        hw.blueExtend.setPosition(0.80);
//        hw.blackExtend.setPosition(0.20);
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);

        hw.blueLift.setTargetPosition((int) targetDistance[4]);
        hw.blackLift.setTargetPosition((int) targetDistance[4]);
        hw.blackLift.setPower(0.95);
        hw.blueLift.setPower(0.95);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        run.reset();
        while (hw.blackLift.getCurrentPosition() < -2505 && !isStopRequested() && run.milliseconds() < 450) {
            BasicTelemetry(hw, xt, yt, correction, drivePower);
            sleep(20);
        }
// Open Grippies
        hw.blueGrip.setPosition(hw.openBlueGrip);
        hw.blackGrip.setPosition(hw.openBlackGrip);

        run.reset();
        while (hw.blackLift.getCurrentPosition() > ((int) targetDistance[4] - 100) && !isStopRequested()) {
            hw.blueLift.setTargetPosition((int) targetDistance[4] - 100);
            hw.blackLift.setTargetPosition((int) targetDistance[4] - 100);
            hw.blackLift.setPower(0.5);
            hw.blueLift.setPower(0.5);
            hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.blueGrip.setPosition(0.05);
            hw.blackGrip.setPosition(0.95);
            BasicTelemetry(hw, xt, yt, correction, drivePower);
            sleep(20);
        }

        while (hw.imuPos.getPosY() > targetDistance[0] && !isStopRequested()) {
            sleep(1);
            hw.frontLeft.setPower(-drivePower + correction);
            hw.frontRight.setPower(-drivePower - correction);
            hw.backLeft.setPower(-drivePower + correction);
            hw.backRight.setPower(-drivePower - correction);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            BasicTelemetry(hw, xt, yt, correction, drivePower);
        }
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);
        hw.blueLift.setTargetPosition(3);
        hw.blackLift.setTargetPosition(3);

        hw.blackLift.setPower(.6);
        hw.blueLift.setPower(.6);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (run.milliseconds() < 100000 && !isStopRequested()) {
            BasicTelemetry(hw, xt, yt, correction, drivePower);
            sleep(20);
        }

    }

    private void BasicTelemetry(Hardware hw, double xt, double yt, double correction, double drivePower) {
        TelemetryHelper.UpdateTelemetry(telemetry,
                "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                "Position X", hw.imuPos.getPosX(),
                "Position Y", hw.imuPos.getPosY(),
                "X", xt,
                "bluePos", hw.blueLift.getCurrentPosition(),
                "blackPos", hw.blackLift.getCurrentPosition(),
                "buGripPos", hw.blueGrip.getPosition(),
                "baGripPos", hw.blackGrip.getPosition(),
                "blueTrgt", hw.blueLift.getTargetPosition(),
                "blackTrgt", hw.blackLift.getTargetPosition(),
                "Y", yt,
                "correction", correction,
                "Power", drivePower
                );
    }

    private double correction(double headingDelta) {
        double mirror = 1.0;
        double correction = 0;
        if (Math.abs(headingDelta) > 1) {
            if (headingDelta > 0)
                correction = 0.1;
            else if (headingDelta < 0)
                correction = -0.1;
            else
                correction = 0;

        }
        return correction;

    }
}
