package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.Units;
import org.firstinspires.ftc.teamcode.BasicPathingMethods;

@Autonomous
public class leftSidedClip extends LinearOpMode {

    // defines run.milliseconds function
    ElapsedTime run = new ElapsedTime();
    BasicPathingMethods path;

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
        path.SetLiftPos(hw, run, liftTPos, true, liftPower);

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
        double correction = path.correction(hDelta);
        path.BasicTelemetry(hw, drivePower);

        // Function that drives it straight
        path.driveStraight(hw, false, TDis, drivePower);

        //Function that drives it strafe
        path.StrafeMilli(hw, run, false, TSec, strafPower);

        // Functions when robot stopped
        run.reset();
        hw.imuPos.update();
        path.BasicTelemetry(hw, drivePower);
        path.stopRobot(hw);

        // Function to set lift position
        path.SetLiftPos(hw, run, -3350, false, .6);

        path.driveStraight(hw, false, 30.2, drivePower);

        path.stopRobot(hw);

        path.SetLiftPos(hw, run, -2480, true, 0.95);

        // Opens Grips
        path.gripPosition(hw, true);

        int raiseLift = 100;
        path.SetLiftPos(hw, run, -2480 - raiseLift, true, 0.65);

        path.driveStraight(hw, true, 17.6, drivePower);

        path.SetLiftPos(hw, run, 3, true, .6);

        path.StrafeMilli(hw, run, true,1350, strafPower);

        path.rotate(hw, true, 83, strafPower);

        double millisecondsPower = 0.5;
        double millisecondsPowerStrafe = 0.5;
        triangle(run, hw, -millisecondsPower, millisecondsPowerStrafe);

        path.stopRobot(hw);
    }

    private void triangle(ElapsedTime run, Hardware hw, double millisecondsPower, double millisecondsPowerStrafe) {
        while (run.milliseconds() < 4300 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        run.reset();
        while (run.milliseconds() < 270 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            path.BasicTelemetry(hw, millisecondsPowerStrafe);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        run.reset();
        while (run.milliseconds() < 450 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            path.BasicTelemetry(hw, millisecondsPowerStrafe);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(millisecondsPower);
            hw.frontRight.setPower(-millisecondsPower);
            hw.backLeft.setPower(-millisecondsPower);
            hw.backRight.setPower(millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        run.reset();
        while (run.milliseconds() < 265 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPowerStrafe);
            hw.frontRight.setPower(-millisecondsPowerStrafe);
            hw.backLeft.setPower(-millisecondsPowerStrafe);
            hw.backRight.setPower(-millisecondsPowerStrafe);
            path.BasicTelemetry(hw, millisecondsPowerStrafe);
        }
        run.reset();
        while (run.milliseconds() < 2500 && !isStopRequested()) {
            hw.frontLeft.setPower(-millisecondsPower);
            hw.frontRight.setPower(millisecondsPower);
            hw.backLeft.setPower(millisecondsPower);
            hw.backRight.setPower(-millisecondsPower);
            path.BasicTelemetry(hw, millisecondsPower);
        }
        path.stopRobot(hw);
    }

}
