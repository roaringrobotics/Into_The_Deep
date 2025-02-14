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
public class leftSidedClip extends LinearOpMode {


    ElapsedTime run = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware adjustments
        Hardware hw = new Hardware(hardwareMap);
        hw.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Variables
        double drivePower = 0.60;
        double strafPower = 0.5;
        double millisecondsPower = 0;
        double liftPower = 0.3;
        int liftTPos = -200;
        double TDis = 20.0;
        double TSec = 500;
        //Set grip positions & power behavior
        hw.blueGrip.setPosition(hw.closeBlueGrip);
        hw.blackGrip.setPosition(hw.closeBlackGrip);


        hw.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(250);

        waitForStart();

        //set up after start
        // Function to set lift position
        SetLiftPos(hw, run, liftTPos, true, liftPower);

        // Resets the timer upon starting
        run.reset();

        // Sets Telemetry and other varibles
        hw.imuPos.reset();
        hw.imuPos.update();
        // to take the delta for positions
        double xt = hw.imuPos.getPosX();
        double yt = hw.imuPos.getPosY();

        double hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
        //fix drifting
        double correction = correction(hDelta);
        BasicTelemetry(hw, correction, drivePower);

        // Function that drives it straight
        driveStraight(hw, false, TDis, drivePower);

        //Function that drives it strafe
        driveStrafeMilli(true, TSec, hw, strafPower);

        // Funtions when robot stopped
        run.reset();
        hw.imuPos.update();
        BasicTelemetry(hw, correction, drivePower);
        stopRobot(hw);

        // Function to set lift position
        SetLiftPos(hw, run, -3350, false, .6);

        driveStraight(hw, false, 30.2, drivePower);

        stopRobot(hw);

        SetLiftPos(hw, run, -2480, true, 0.95);

        // Opens Grips
        gripPosition(hw, true);

        int raiseLift = 100;
        SetLiftPos(hw, run, -2480 - raiseLift, true, 0.65);

        driveStraight(hw, true, 17.6, drivePower);

        SetLiftPos(hw, run, 3, true, .6);

        driveStrafeMilli(false,1350, hw, strafPower);

        rotate(hw, true, 83, strafPower);

        millisecondsPower = 0.5;
        double millisecondsPowerStrafe = 0.5;
        triangle(run, hw, -millisecondsPower, millisecondsPowerStrafe);

        stopRobot(hw);
    }

    private static void stopRobot(Hardware hw) {
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);
    }

    // all defined functions:
    private void rotate(Hardware hw, boolean isClockwise, double targetDegrees, double strafPower) {
        double correction = 0;
        if (isClockwise)
            targetDegrees *= -1;
        else
            strafPower *= -1;

        while (hw.imuPos.getHeading(Units.AngularUnit.Degree) > targetDegrees && !isStopRequested()) {
            sleep(1);
            hw.frontLeft.setPower(strafPower);
            hw.frontRight.setPower(-strafPower);
            hw.backLeft.setPower(strafPower);
            hw.backRight.setPower(-strafPower);

            hw.imuPos.update();
            BasicTelemetry(hw, correction, strafPower);
        }

    }

    private static void gripPosition(Hardware hw, boolean isGripOpen) {
        if (isGripOpen) {
            hw.blueGrip.setPosition(hw.openBlueGrip);
            hw.blackGrip.setPosition(hw.openBlackGrip);
        } else {
            hw.blueGrip.setPosition(hw.closeBlueGrip);
            hw.blackGrip.setPosition(hw.closeBlackGrip);
        }
    }

    private void driveStrafeMilli(boolean isRight, double TargetMilliseconds, Hardware hw, double strafPower) {
        double hDelta;
        double correction = 0;
        run.reset();
        if (isRight) {
            while (run.milliseconds() < TargetMilliseconds && !isStopRequested()) {
                hw.frontLeft.setPower(strafPower);
                hw.frontRight.setPower(-strafPower);
                hw.backLeft.setPower(-strafPower);
                hw.backRight.setPower(strafPower);
                hw.imuPos.update();
                hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
                correction = correction(hDelta);
                BasicTelemetry(hw, correction, strafPower);
            }
        } else {
            while (run.milliseconds() < TargetMilliseconds && !isStopRequested()) {
                hw.frontLeft.setPower(-strafPower);
                hw.frontRight.setPower(strafPower);
                hw.backLeft.setPower(strafPower);
                hw.backRight.setPower(-strafPower);
                hw.imuPos.update();
                hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
                correction = correction(hDelta);
                BasicTelemetry(hw, correction, strafPower);
            }
        }
    }

    private void driveStraight(Hardware hw, boolean isBackwards, double TargetDistance, double drivePower) {
        double hDelta;
        double correction = 0;
        if (!isBackwards) {
            while (Math.abs(hw.imuPos.getPosY()) < Math.abs(TargetDistance) && !isStopRequested()) {
                sleep(1);
                hw.frontLeft.setPower(drivePower + correction);
                hw.frontRight.setPower(drivePower - correction);
                hw.backLeft.setPower(drivePower + correction);
                hw.backRight.setPower(drivePower - correction);
                hw.imuPos.update();
                hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
                correction = correction(hDelta);
                BasicTelemetry(hw, correction, drivePower);
            }
        } else {
            while (Math.abs(hw.imuPos.getPosY()) > Math.abs(TargetDistance) && !isStopRequested()) {
                sleep(1);
                hw.frontLeft.setPower(-drivePower + correction);
                hw.frontRight.setPower(-drivePower - correction);
                hw.backLeft.setPower(-drivePower + correction);
                hw.backRight.setPower(-drivePower - correction);
                hw.imuPos.update();
                hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
                correction = correction(hDelta);
                BasicTelemetry(hw, correction, drivePower);}
        }
    }

    private void SetLiftPos(Hardware hw, ElapsedTime run, int liftTargetPosition, boolean isExpectedtoLoop, double liftPower) {
        hw.blueLift.setTargetPosition(liftTargetPosition);
        hw.blackLift.setTargetPosition(liftTargetPosition);
        hw.blackLift.setPower(liftPower);
        hw.blueLift.setPower(liftPower);
        hw.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        run.reset();
        // isExpectedtoLoop is so that if the lift is expected to get stuck, there is another guideline.
        if (isExpectedtoLoop) {
            while (hw.blackLift.getCurrentPosition() < -liftTargetPosition && !isStopRequested() && run.milliseconds() < 1000) {
                BasicTelemetry(hw, 0, liftPower);
                sleep(20);
            }
            } else
            while (hw.blackLift.getCurrentPosition() < -liftTargetPosition -20 && !isStopRequested()) {
                BasicTelemetry(hw, 0, liftPower);
                sleep(20);
        }
    }

    private void triangle(ElapsedTime run, Hardware hw, double millisecondsPower, double millisecondsPowerStrafe) {
        while (run.milliseconds() < 4300 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 270 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 450 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 265 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            BasicTelemetry(hw);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            BasicTelemetry(hw);
        }
        stopRobot(hw);
    }

    private void BasicTelemetry(Hardware hw, double correction, double drivePower) {
        TelemetryHelper.UpdateTelemetry(telemetry,
                "bot heading", hw.imuPos.getHeading(Units.AngularUnit.Degree),
                "Position X", hw.imuPos.getPosX(),
                "Position Y", hw.imuPos.getPosY(),
                "bluePos", hw.blueLift.getCurrentPosition(),
                "blackPos", hw.blackLift.getCurrentPosition(),
                "buGripPos", hw.blueGrip.getPosition(),
                "baGripPos", hw.blackGrip.getPosition(),
                "blueTrgt", hw.blueLift.getTargetPosition(),
                "blackTrgt", hw.blackLift.getTargetPosition(),
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

    private void BasicTelemetry(Hardware hw) {
        hw.imuPos.update();
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
