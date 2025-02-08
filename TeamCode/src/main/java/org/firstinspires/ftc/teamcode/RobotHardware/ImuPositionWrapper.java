package org.firstinspires.ftc.teamcode.RobotHardware;

import org.firstinspires.ftc.teamcode.Units;

// Interface for accessing hardware that provides
// header and position information.
// Implemented by ImuImpl and PinpointImpl.
public interface ImuPositionWrapper {
    // Reset the IMU heading and position if possible.
    void reset() throws InterruptedException;

    void resetHeading() throws InterruptedException;

    // Get heading in degrees.
    double getHeading(Units.AngularUnit units);
    // Get Y position in inches.  +Y is forward -Y is backward relative to robot.
    double getPosY();
    // Get X position in inches.  +X is left -X is right relative to robot.
    double getPosX();
    // Called to read information from hardware if needed.
    // Users of ImuPositionWrapper should always call this before reading new values.
    void update();
}

