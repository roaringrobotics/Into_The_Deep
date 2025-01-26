package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

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
    public void reset() {
        pp.resetPosAndIMU();
    }

    @Override
    public void resetHeading()
    {
        pp.recalibrateIMU();
    }

    @Override
    public double getHeading(AngleUnit units) {
        if(units == AngleUnit.DEGREES)
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

    @Override
    public Pose2D getPose() {
        return pp.getPosition();
    }

}
