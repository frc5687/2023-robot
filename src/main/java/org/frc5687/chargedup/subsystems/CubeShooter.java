package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

import edu.wpi.first.math.util.Units;

public class CubeShooter extends OutliersSubsystem {
    private final OutliersTalon _wrist;
    private final OutliersTalon _shooter;

   private CubeShooter(OutliersContainer container){
    super(container);
    _wrist = new OutliersTalon(RobotMap.CAN.TALONFX.CUBESHOOTER_WRIST, Constants.CubeShooter.CAN_BUS, "Cube Shooter Wrist");
    _shooter = new OutliersTalon(RobotMap.CAN.TALONFX.CUBESHOOTER_SHOOTER, Constants.CubeShooter.CAN_BUS, "Cube Shooter Shooter");
    _wrist.configure(Constants.CubeShooter.WRIST_CONFIG);
    _wrist.configureClosedLoop(Constants.CubeShooter.CONTROLLER_CONFIG);
    _shooter.configure(Constants.CubeShooter.SHOOTER_CONFIG);
   } 

   public void setWristSpeed(double speed){
    _wrist.setPercentOutput(speed);
   }

   public void setShooterSpeed(double speed){
    _shooter.setPercentOutput(speed);
   }

   public void setWristPosition(double rotation){
    _wrist.setMotionMagic(rotation);
   }

   public void setWristEncoderRotation(double rotation){
    _wrist.setRotorPosition(rotation);
   }

   public double getWristCurrent(){
    return _wrist.getStatorCurrent().getValue();
   }

   public double getWristEncoderRotation(){
    return _wrist.getPosition().getValue();
   }

   public double getWristAngleRadians(){
    return Units.rotationsToRadians(getWristEncoderRotation());
   }

   public void updateDashboard() {
   }
}
