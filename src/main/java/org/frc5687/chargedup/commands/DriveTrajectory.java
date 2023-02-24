/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.DriveTrain.ControlState;
import org.frc5687.lib.math.Vector2d;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Trajectory _trajectory;
    private final Timer _timer;
    private Translation2d _cor;
    private double _rotfrompos;

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory, double rotfrompos) {
        _driveTrain = driveTrain;
        _timer = new Timer();
        _trajectory = trajectory;
        _rotfrompos = rotfrompos;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setControlState(ControlState.TRAJECTORY);
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        // double
        Trajectory.State goal = _trajectory.sample(_timer.get());
        Translation2d cor = _driveTrain.getDesiredCOR(_rotfrompos);
        _driveTrain.updateSwerve(goal, new Rotation2d(0.0), cor);
    }

    @Override
    public boolean isFinished() {
        return _timer.get() >= _trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.setControlState(ControlState.MANUAL);
        _driveTrain.updateSwerve(Vector2d.identity(), 0, new Translation2d());
        _timer.reset();
    }
}
