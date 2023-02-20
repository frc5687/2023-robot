package org.frc5687.config;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class EvasiveManeuverPoses {
    public EvasiveManeuverPoses(DriveTrain driveTrain){
    }

    /* The pose would be 
     * (robotpose.x + 1 meter * sin(_oi.getHeading())), (robotpose.y + 1 meter * cos(_oi.getHeading()))
     * 
     * 
     * 
     */
}
