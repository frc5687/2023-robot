package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class ResetRobotPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final Pose2d _pose;
    public ResetRobotPose(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _pose = pose;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.resetRobotPose(_pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
