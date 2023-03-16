/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.DriveTrain.ControlState;
import org.frc5687.lib.math.Vector2d;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Trajectory _trajectory;
    private final Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory) {
        _driveTrain = driveTrain;
        _timer = new Timer();
        _trajectory = trajectory;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
        _driveTrain.setControlState(ControlState.TRAJECTORY);
        _driveTrain.setKinematicLimits(Constants.DriveTrain.TRAJECTORY_FOLLOWING);
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        Trajectory.State goal = _trajectory.sample(_timer.get());
        _driveTrain.followTrajectory(goal, new Rotation2d());
    }

    @Override
    public boolean isFinished() {
        return _timer.get() >= _trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.setControlState(ControlState.MANUAL);
        _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
        _driveTrain.setVelocity(new ChassisSpeeds());
        _timer.reset();
    }
}
