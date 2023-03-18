/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.chargedup.subsystems.DriveTrain;

import java.util.function.Consumer;

/** Custom PathPlanner version of SwerveControllerCommand */
public class DriveTrajectory extends OutliersCommand {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final boolean useAllianceColor;
    private PathPlannerTrajectory transformedTrajectory;
    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;

    private final DriveTrain _driveTrain;

    public DriveTrajectory(
            DriveTrain driveTrain,
            PathPlannerTrajectory trajectory,
            boolean useAllianceColor) {

        _driveTrain = driveTrain;
        this.trajectory = trajectory;
        this.useAllianceColor = useAllianceColor;
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
        if (useAllianceColor && trajectory.fromGUI) {
            transformedTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            trajectory, DriverStation.getAlliance());
        } else {
            transformedTrajectory = trajectory;
        }

        if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(transformedTrajectory);
        }

        timer.reset();
        timer.start();

        PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);
        _driveTrain.followTrajectory(desiredState);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();

        if (interrupted || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
            _driveTrain.setVelocity(new ChassisSpeeds(
                    0,
                    0,
                    0
            ));
        }
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
    }

    private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(
                "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
    }
}
