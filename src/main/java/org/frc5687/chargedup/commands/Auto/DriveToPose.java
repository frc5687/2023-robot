package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;

import static org.frc5687.chargedup.Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS;
import static org.frc5687.chargedup.Constants.DriveTrain.KINEMATIC_LIMITS;

public class DriveToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Pose2d _destPose;
    private final OI _oi;

    public DriveToPose(DriveTrain driveTrain, Pose2d pose, OI oi) {
        _driveTrain = driveTrain;
        _destPose = pose;
        _oi = oi;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setKinematicLimits(DRIVE_POSE_KINEMATIC_LIMITS);
        //        _destPose = new Pose2d(_destPose.getX(), _destPose.getY(), _driveTrain.getHeading());
    }

    @Override
    public void execute() {
        _driveTrain.setVelocityPose(_destPose);
    }

    @Override
    public boolean isFinished() {
        double xDiff = _destPose.getX() - _driveTrain.getEstimatedPose().getX();
        double yDiff = _destPose.getY() - _driveTrain.getEstimatedPose().getY();
        return (Math.abs(xDiff) < 0.03 && Math.abs(yDiff) < 0.03)
         || (_oi.getDriveX() > 0 || _oi.getDriveY() > 0) /* || 
         (_oi.getTapRight() || _oi.getTapLeft()) */;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setKinematicLimits(KINEMATIC_LIMITS);
        super.end(interrupted);
    }
}
