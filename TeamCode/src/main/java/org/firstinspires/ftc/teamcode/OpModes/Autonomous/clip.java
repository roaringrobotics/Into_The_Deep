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


        hw.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime run = new ElapsedTime();
        double drivePower = 0.20;
        double[] targetDistance = {
                26,
                -5.0
        };

        waitForStart();

        // Resets the timer upon starting
//        run.reset();
        hw.imuPos.reset();
        double xt = hw.imuPos.getPosX();
        double yt = hw.imuPos.getPosY();
        double hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
        double correction = correction(hDelta);
        sleep(250);
        // While loop that moves it straight
        while (hw.imuPos.getPosY() < targetDistance[0] && !isStopRequested()) {
            hw.frontLeft.setPower(drivePower + correction);
            hw.frontRight.setPower(drivePower - correction);
            hw.backLeft.setPower(drivePower + correction);
            hw.backRight.setPower(drivePower - correction);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            TelemetryHelper.UpdateTelemetry(telemetry,
                    "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                    "Position X", hw.imuPos.getPosX(),
                    "Position Y", hw.imuPos.getPosY(),
                    "X", xt,
                    "bluePos", hw.blueLift.getCurrentPosition(),
                    "blackPos", hw.blackLift.getCurrentPosition(),
                    "blueTrgt", hw.blueLift.getTargetPosition(),
                    "blackTrgt", hw.blackLift.getTargetPosition(),
                    "Y", yt,
                    "correction", correction);
        }
//        run.reset();
        //starfe right
        while (hw.imuPos.getPosX() > targetDistance[1] && !isStopRequested()) {
            hw.frontLeft.setPower(-drivePower - correction);
            hw.frontRight.setPower(drivePower - correction);
            hw.backLeft.setPower(drivePower + correction);
            hw.backRight.setPower(-drivePower + correction);
            hw.imuPos.update();
            hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
            correction = correction(hDelta);
            TelemetryHelper.UpdateTelemetry(telemetry,
                    "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                    "Position X", hw.imuPos.getPosX(),
                    "Position Y", hw.imuPos.getPosY(),
                    "X", xt,
                    "bluePos", hw.blueLift.getCurrentPosition(),
                    "blackPos", hw.blackLift.getCurrentPosition(),
                    "blueTrgt", hw.blueLift.getTargetPosition(),
                    "blackTrgt", hw.blackLift.getTargetPosition(),
                    "Y", yt,
                    "correction", correction);
        }
        run.reset();
        hw.imuPos.update();
        TelemetryHelper.UpdateTelemetry(telemetry,
                "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                "Position X", hw.imuPos.getPosX(),
                "Position Y", hw.imuPos.getPosY(),
                "X", xt,
                "bluePos", hw.blueLift.getCurrentPosition(),
                "blackPos", hw.blackLift.getCurrentPosition(),
                "blueTrgt", hw.blueLift.getTargetPosition(),
                "blackTrgt", hw.blackLift.getTargetPosition(),
                "Y", yt);
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);
        //rotate
        hw.blueLift.setTargetPosition(-3000);
        hw.blackLift.setTargetPosition(-3000);

        hw.blackLift.setPower(.3);
        hw.blueLift.setPower(.3);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Wait until there
        while (hw.blackLift.getCurrentPosition() > -3150 && !isStopRequested()) {
            sleep(1100);
            hw.frontLeft.setPower(0.09);
            hw.frontRight.setPower(0.09);
            hw.backLeft.setPower(0.09);
            hw.backRight.setPower(0.09);
        }

        hw.blueLift.setTargetPosition(0);
        hw.blackLift.setTargetPosition(0);

        hw.blackLift.setPower(.3);
        hw.blueLift.setPower(.3);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hw.blackLift.getCurrentPosition() < 0 && !isStopRequested()) {
            sleep(10);

        }
        run.reset();


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
