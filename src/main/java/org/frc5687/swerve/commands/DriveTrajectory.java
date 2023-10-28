/* Team 5687 (C)2021-2022 */
package org.frc5687.swerve.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Consumer;

import org.frc5687.swerve.Constants;
import org.frc5687.swerve.subsystems.DriveTrain;

/** Custom PathPlanner version of SwerveControllerCommand */
public class DriveTrajectory extends OutliersCommand {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final boolean _resetRobotPose;
    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;

    private final DriveTrain _driveTrain;

    public DriveTrajectory(
            DriveTrain driveTrain,
            PathPlannerTrajectory trajectory,
            boolean useAllianceColor,
            boolean resetRobotPose) {

        _driveTrain = driveTrain;
        this.trajectory = trajectory;
    
        _resetRobotPose = resetRobotPose;

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
//        if (useAllianceColor && trajectory.fromGUI) {
//            error(" Transforming Trajectory");
//            transformedTrajectory =
//                    PathPlannerTrajectory.transformTrajectoryForAlliance(
//                            trajectory, DriverStation.getAlliance());
//        } else {
//            transformedTrajectory = trajectory;
//        }
        if (_resetRobotPose) {
            //_driveTrain.wantsToResetPose(trajectory.getInitialHolonomicPose());
        }

        if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(trajectory);
        }

        timer.reset();
        timer.start();

        PathPlannerServer.sendActivePath(trajectory.getStates());

        _driveTrain.setKinematicLimits(Constants.DriveTrain.TRAJECTORY_FOLLOWING);
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(currentTime);
        _driveTrain.followTrajectory(desiredState);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();

        if (interrupted
                || Math.abs(trajectory.getEndState().velocityMetersPerSecond) < 0.1) {
            _driveTrain.setVelocity(new ChassisSpeeds(0, 0, 0));
            _driveTrain.setKinematicLimits(_driveTrain.isLowGear() ? Constants.DriveTrain.LOW_KINEMATIC_LIMITS :
         Constants.DriveTrain.HIGH_KINEMATIC_LIMITS);
        }
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
