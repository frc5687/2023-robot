package org.frc5687.chargedup.commands.Auto;

import java.util.function.Consumer;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajePose extends OutliersCommand{
    private Timer timer = new Timer();
    private final DriveTrain _driveTrain;
    private Pose2d _destPose;
    private PathPlannerTrajectory _trajectory;
    private boolean _isShooter;
    private boolean _resetRobotPose;
    private boolean _invertPose; // If false, will drive to pose when closer to driver station. If true, will drive to pose when farther to driver station.
    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;


    public DriveTrajePose(DriveTrain driveTrain, 
            PathPlannerTrajectory trajectory, 
            boolean useAllianceColor, 
            boolean resetRobotPose,
            Pose2d destPose,
            boolean isShooter, boolean invertPose){
        _driveTrain = driveTrain;
        _trajectory = trajectory;
        _resetRobotPose = resetRobotPose;
        _destPose = destPose;
        _isShooter = isShooter;
        _invertPose = invertPose;
        addRequirements(driveTrain);
                
        if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
            DriverStation.reportWarning(
                    "You have constructed a path following command that will automatically transform path states depending"
                        + " on the alliance color, however, it appears this path was created on the red side of the field"
                        + " instead of the blue side. This is likely an error.",
                    false);
                }
    }    

    @Override
    public void initialize() {
        if (_resetRobotPose) {
            _driveTrain.wantsToResetPose(_trajectory.getInitialHolonomicPose());
        }

        if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(_trajectory);
        }

        timer.reset();
        timer.start();

        PathPlannerServer.sendActivePath(_trajectory.getStates());

        _driveTrain.setKinematicLimits(Constants.DriveTrain.TRAJECTORY_FOLLOWING);
    }

    @Override
    public void execute() {
        if (_driveTrain.isRedAlliance() 
        ? (_invertPose ? _driveTrain.getEstimatedPose().getX() > Constants.Auto.RED_X_TRAJ_END_COORDINATE : _driveTrain.getEstimatedPose().getX() < Constants.Auto.RED_X_TRAJ_END_COORDINATE)
        : (_invertPose ? _driveTrain.getEstimatedPose().getX() < Constants.Auto.BLUE_X_TRAJ_END_COORDINATE : _driveTrain.getEstimatedPose().getX() > Constants.Auto.BLUE_X_TRAJ_END_COORDINATE)){
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) _trajectory.sample(currentTime);
        _driveTrain.followTrajectory(desiredState);
        } else {
            _driveTrain.setKinematicLimits(Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS);
            _driveTrain.setVelocityPose(_destPose, _isShooter);
        }
    }

    @Override
    public boolean isFinished() {
        double xDiff = _destPose.getX() - _driveTrain.getEstimatedPose().getX();
        double yDiff = _destPose.getY() - _driveTrain.getEstimatedPose().getY();
        return (Math.abs(xDiff) < 0.03 && Math.abs(yDiff) < 0.03);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
    }
}
