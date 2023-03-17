package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class SetHoverGoal extends OutliersCommand {
    private DriveTrain _driveTrain;
    private Pose2d _goal;

    public SetHoverGoal(DriveTrain driveTrain, Pose2d goal) {
        _driveTrain = driveTrain;
        _goal = goal;
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setHoverGoal(_goal);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
