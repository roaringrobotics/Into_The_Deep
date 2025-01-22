package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Units;
import org.firstinspires.ftc.teamcode.Units.AngularUnit;

@TeleOp
public class HighTel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);
        FieldCentricPowerLevels fieldCentricPowerLevels = new FieldCentricPowerLevels();

        initializeExtenderGripPositions();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controlLift();
            controlExtender();
            controlGrip();
            controlResetImu();
            double powerScale = controlScalePower();
            driveFieldCentric(
                    -gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    powerScale, hw,
                    fieldCentricPowerLevels);
            updateTelemetry();
        }
    }

    private void initializeExtenderGripPositions() {
        hw.blackGrip.setPosition(0.5);
        hw.blueGrip.setPosition(0.5);
        hw.blackExtend.setPosition(0.1);
        hw.blueExtend.setPosition(0.9);
    }

    private void controlLift() {
        // Read joystick input to control lift
        int bluePos = hw.blueLift.getCurrentPosition();

        // Safe values for encoder positions
        // based on manual testing of robot.
        // If these are wrong it can break the robot.
        boolean downSafe = bluePos < 3;
        boolean upSafe = bluePos > -3315;

        if (gamepad2.dpad_up && upSafe) {
            hw.blueLift.setPower(-0.5);
            hw.blackLift.setPower(-0.5);
        } else if (gamepad2.dpad_down && downSafe) {
            hw.blueLift.setPower(0.5);
            hw.blackLift.setPower(0.5);
        } else {
            hw.blueLift.setPower(-0.08);
            hw.blackLift.setPower(-0.08);
        }
    }

    private void updateTelemetry() {
        hw.imuPos.update();
        double botHeading = hw.imuPos.getHeading(AngularUnit.Degree);
        double xo = hw.imuPos.getPosX();
        double yo = hw.imuPos.getPosY();
        double xt = startEncX - hw.imuPos.getPosX();
        double yt = startEncY - hw.imuPos.getPosY();
        double bluePos = hw.blueLift.getCurrentPosition();

        // Show the position of the motor on telemetry
        TelemetryHelper.UpdateTelemetry(telemetry,
                "bot heading", botHeading,
                "Position X", xo,
                "Position Y", yo,
                "X", xt,
                "bluePos", bluePos,
                "Y", yt);


    }

    private void controlResetImu() {
        // Reset imu if options is pressed.
        if (gamepad1.options) {
            hw.imuPos.reset();
        }
    }

    private void driveFieldCentric(double leftStickY, double leftStickX, double rightStickX,
                                   double powerScale, Hardware hw,
                                   FieldCentricPowerLevels fcPowerLevels) {
        // Takes joystick input and sets motor power levels to drive
        // robot field centric.
        hw.imuPos.update();
        double botHeading = hw.imuPos.getHeading(AngularUnit.Radian);

        getFieldCentricPowerLevels(
                leftStickY, leftStickX,
                rightStickX, botHeading,
                fcPowerLevels);

        hw.frontLeft.setPower(fcPowerLevels.frontLeftPower * powerScale);
        hw.backLeft.setPower(fcPowerLevels.backLeftPower * powerScale);
        hw.frontRight.setPower(fcPowerLevels.frontRightPower * powerScale);
        hw.backRight.setPower(fcPowerLevels.backRightPower * powerScale);
    }

    private static void getFieldCentricPowerLevels(
            double leftStickY, double leftStickX,
            double rightStickX, double botHeading,
            FieldCentricPowerLevels fcPowerLevels) {

        // leftStickY forward/backward [-1.0, 1.0]
        // leftStickX strafe [-1.0, 1.0]
        // rightStickX rotate [-1.0, 1.0].

        // Rotate joystick input vectors for field centric control.

        // Apply rotation matrix inverse of botHeading (-botHeading).
        // | cos(-h) -sin(-h) | | lsx  lsy |
        // | sin(-h)  cos(-h) | | lsx  lsy |
        // h = botHeading
        // lsx = left stick x value
        // lsy = left stick y value.
        double rotatedX = leftStickX * Math.cos(-botHeading) - leftStickY * Math.sin(-botHeading);
        double rotatedY = leftStickX * Math.sin(-botHeading) + leftStickY * Math.cos(-botHeading);

        // Don't know why the rotated value of left stick x is scale by 1.1.
        // It's not in online example code.
        rotatedX = rotatedX * 1.1;

        // Normalize output power [-1.0-1.0]
        double vectorSum = Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rightStickX);
        double normalize = 1.0 / Math.max(vectorSum, 1.0);

        // Set normalized field centric power levels.
        fcPowerLevels.frontLeftPower = (rotatedY + rotatedX + rightStickX) * normalize;
        fcPowerLevels.backLeftPower = (rotatedY - rotatedX + rightStickX) * normalize;
        fcPowerLevels.frontRightPower = (rotatedY - rotatedX - rightStickX) * normalize;
        fcPowerLevels.backRightPower = (rotatedY + rotatedX - rightStickX) * normalize;
    }

    // Class to hold field centric power level output from getFieldCentricPowerLevels
    private static class FieldCentricPowerLevels {
        public double frontLeftPower;
        public double backLeftPower;
        public double frontRightPower;
        public double backRightPower;

        public FieldCentricPowerLevels() {

        }
    }

    private double controlScalePower() {
        // Check joystick
        // scale power if bumpers are pressed.
        double powerScale;
        if (gamepad1.right_bumper) {
            powerScale = 0.25;
        } else if (gamepad1.left_bumper) {
            powerScale = 0.5;
        } else {
            powerScale = 1.0;
        }
        return powerScale;
    }

    private void controlGrip() {
        // Check joystick to control grip.
        // using gamepad 2.
        if (gamepad2.b) {
            hw.blueGrip.setPosition(1);
            hw.blackGrip.setPosition(0);
        } else if (gamepad2.a) {
            hw.blueGrip.setPosition(0);
            hw.blackGrip.setPosition(1);
        }
    }

    private void controlExtender() {
        // Check joystick to control extender
        // using gamepad 2.
        if (gamepad2.dpad_right) {
            hw.blueExtend.setPosition(0.05);
            hw.blackExtend.setPosition(0.95);
        } else if (gamepad2.dpad_left) {
            hw.blueExtend.setPosition(0.95);
            hw.blackExtend.setPosition(0.05);
        }
    }

    Hardware hw;

    // following are only used to track how far moved for telemetry.
    double startEncX = 0;
    double startEncY = 0;

    ////////////

}

