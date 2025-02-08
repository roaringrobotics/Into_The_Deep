package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.RobotHardware.ImuPositionWrapper;
import org.firstinspires.ftc.teamcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Units;

@TeleOp
public class manualAutoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(hardwareMap);


        hw.imuPos.reset();
        double xt = hw.imuPos.getPosX();
        double yt = hw.imuPos.getPosY();
        double hDelta = hw.imuPos.getHeading(Units.AngularUnit.Degree);
        double correction = correction(hDelta);
        sleep(250);

        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.options)
                hw.imuPos.reset();

            BasicTelemetry(hw, xt, yt, correction);
            sleep(1);
        }
    }








    //classes
    private void BasicTelemetry(Hardware hw, double xt, double yt, double correction) {
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
