package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.subsystems.*;

public class OneAndAHalfLevel extends SequentialCommandGroup {
    public OneAndAHalfLevel(
            DriveTrain driveTrain, Arm arm, Elevator elevator, EndEffector endEffector, CubeShooter shooter) {
        DriverStation.reportError("Starting cone and level auto", false);
        addCommands(
                new SequentialCommandGroup(
                        Commands.runOnce(endEffector::setConeState, endEffector),
                        new AutoPlaceAndStowHighCone(elevator, endEffector, arm),
                        new LevelingAndIntake(driveTrain, shooter)));
    }
}

