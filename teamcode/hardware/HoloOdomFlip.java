package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import java.util.function.DoubleSupplier;

// odometry device with swapped x and y components
public class HoloOdomFlip extends HolonomicOdometry {

    // copy private declaration from parent class
    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private double centerWheelOffset;

    // same constructor format
    public HoloOdomFlip(
            DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
            DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {

        // call super constructor and manually initialize variables on this level
        super (leftEncoder, rightEncoder, horizontalEncoder, trackWidth, centerWheelOffset);
        previousAngle = robotPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
    }

    // copy base class update method and edit to function with variables of this access level
    @Override
    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {

        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / trackWidth
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        // swapped x and y components
        double dy = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dx = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}
