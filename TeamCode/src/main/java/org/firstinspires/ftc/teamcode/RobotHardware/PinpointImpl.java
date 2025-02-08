package org.firstinspires.ftc.teamcode.RobotHardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Units;

// ImuPostionWrapper implementation for GoBilda Pinpoint computer.
public class PinpointImpl implements ImuPositionWrapper {
    private final GoBildaPinpointDriver pp;

    public PinpointImpl(HardwareMap hwmap) {
        pp = hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pp.setEncoderResolution(Constants.encoderResolution);
    }

    @Override
    public void reset() throws InterruptedException {
        pp.resetPosAndIMU();
        sleep(500);
    }

    @Override
    public void resetHeading() throws InterruptedException {
        pp.recalibrateIMU();
        sleep(500);
    }

    @Override
    public double getHeading(Units.AngularUnit units) {
        if(units == Units.AngularUnit.Degree)
            return Math.toDegrees(pp.getHeading());
        else
            return pp.getHeading();
    }

    @Override
    public double getPosY() {
        return pp.getPosY() * Constants.mmToInch;
    }

    @Override
    public double getPosX() {
        return pp.getPosX() * Constants.mmToInch;
    }

    @Override
    public void update() {
        pp.update();
    }

}
