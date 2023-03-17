import org.frc5687.chargedup.subsystems.DriveTrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowPathCommand extends OutliersCommand{
    DriveTrain _driveTrain;
    FollowPathCommand(PathPlannerTrajectory traj, DriveTrain driveTrain){
        _driveTrain = driveTrain;
        new PPSwerveControllerCommand(
            traj, 
            _driveTrain.getOdometryPose(), // Pose supplier
            _driveTrain.getKinematics(), // SwerveDriveKinematics
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            _driveTrain.setModuleStates(DriveTrain.SystemIO.measuredStates), // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            _driveTrain // Requires this drive subsystem
        );
    }
}