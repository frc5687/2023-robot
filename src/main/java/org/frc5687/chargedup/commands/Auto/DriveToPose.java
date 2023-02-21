package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class DriveToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Pose2d _destPose;

    public DriveToPose(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _destPose = pose;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _destPose = new Pose2d(_destPose.getX(), _destPose.getY(), _driveTrain.getHeading());
    }

    @Override
    public void execute() {
        _driveTrain.setVelocityPose(_destPose);
    }

    @Override
    public boolean isFinished() {
        double xDiff = _destPose.getX() - _driveTrain.getEstimatedPose().getX();
        double yDiff = _destPose.getY() - _driveTrain.getEstimatedPose().getY();
        return Math.abs(xDiff) < 0.1 && Math.abs(yDiff) < 0.1;
    }
}
