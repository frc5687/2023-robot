package org.frc5687.chargedup.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import com.ctre.phoenix.led.*;

public class Lights extends OutliersSubsystem{

    private final CANdle _candle;
    private final CANdleConfiguration _config;
    private Animation _animate;
    private AnimationType _currentAnimation;
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    private OI _oi;
    private int[] _color;

    private boolean _dirty  = true;
    private boolean _slow = false;

    public Lights(OutliersContainer _container, DriveTrain driveTrain, EndEffector endEffector, OI oi) {
        super(_container);
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        _oi = oi;
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT, "rio");
        _config = new CANdleConfiguration();
        // Set LED strip type
        _config.stripType = LEDStripType.RGB;
        // Sets LED brightness
        _config.brightnessScalar = Constants.CANdle.BRIGHTNESS;
        _candle.configAllSettings(_config);
        // set the _color in the constructor to make sure it's not null;
        _color = Constants.CANdle.RED;
        setColor(Constants.CANdle.GREEN);
        switchAnimation(AnimationType.STATIC);
    }

    // Set the color of the lights

    public void setColor(int[] color) {
        if (_color != null &&  !colorsMatch(color, _color))  {
            _color = color;
            _dirty = true;
        }
    }

    public int[] getColor() {
        return _color;
    }

    private boolean colorsMatch(int[] color1, int[] color2) {
        if (color1[0] != color2[0]) { return false;}
        if (color1[1] != color2[1]) { return false;}
        if (color1[2] != color2[2]) { return false;}
        return true;
    }

    /**
     * Switch the current animation to the parameter.
     *
     * @param animation
     */
    public void switchAnimation(AnimationType animation) {
        if (_currentAnimation == animation && !_dirty) {
            return;
        }
        _currentAnimation = animation;
        switch (animation) {
            case COLOR_FLOW:
                _animate =
                        new ColorFlowAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                ColorFlowAnimation.Direction.Forward);
                break;
            case FIRE:
                _animate =
                        new FireAnimation(
                                Constants.CANdle.BRIGHTNESS,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                0.5,
                                0.5);
                break;
            case RAINBOW:
                _animate =
                        new RainbowAnimation(
                                Constants.CANdle.BRIGHTNESS, Constants.CANdle.SPEED, Constants.CANdle.NUM_LED);
                break;
            case STROBE:
                _animate =
                        new StrobeAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED);
                break;
            case LARSON:
                _animate = new LarsonAnimation(_color[0], _color[1], _color[2]);
                break;
            case RGB_FADE:
                _animate =
                        new RgbFadeAnimation(
                                Constants.CANdle.BRIGHTNESS, Constants.CANdle.SPEED, Constants.CANdle.NUM_LED);
                break;
            case SINGLE_FADE:
                _animate =
                        new SingleFadeAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED);
                break;
            case TWINKLE:
                _animate =
                        new TwinkleAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                Constants.CANdle.TWINKLEPERCENT);
                break;
            case TWINKLE_OFF:
                _animate =
                        new TwinkleOffAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                Constants.CANdle.TWINKLEOFFPERCENT);
                break;
            case STATIC:
                _animate = null;
                break;
        }
        _dirty = true;
    }

    /** Has all the logic for the lights, and updates the CANdle with animations and static colors. */
    @Override
    public void periodic() {

        /* if (_oi.rainbow()) {
            switchAnimation(AnimationType.RAINBOW);
        } else if (_endEffector.getConeMode()) {
            switchAnimation(AnimationType.STATIC);
            setColor(Constants.CANdle.YELLOW);
        } else {
            switchAnimation(AnimationType.STATIC);
            setColor(Constants.CANdle.PURPLE);
        } 
    
        // } else if (_driveTrain.isLevel()) {
        //     switchAnimation(AnimationType.STATIC);
        //     setColor(Constants.CANdle.PURPLE);
        // } else {
        //     switchAnimation(AnimationType.FIRE);
        // } */
        if (!_dirty) {
            return;
        }
        
        if (_animate == null) {
            _candle.clearAnimation(0); // very important
            _candle.setLEDs(_color[0], _color[1], _color[2]);
        } else {
            _candle.animate(_animate, 0);
        } 
        _dirty = false;

    } 

    public void updateDashboard() {}

    public enum AnimationType {
        COLOR_FLOW(0),
        FIRE(1),
        RAINBOW(2),
        STROBE(3),
        LARSON(4),
        RGB_FADE(5),
        SINGLE_FADE(6),
        TWINKLE(7),
        TWINKLE_OFF(8),
        STATIC(9);

        private int _value;

        AnimationType(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
