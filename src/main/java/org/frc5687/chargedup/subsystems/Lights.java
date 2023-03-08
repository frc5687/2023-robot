package org.frc5687.chargedup.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;


public class Lights extends OutliersSubsystem{

    private final CANdle _candle;
    private final CANdleConfiguration _config;
    private Animation[] _animate;
    private AnimationType _currentAnimation;
    private int _slots;
    private int[][] _colors;
    private int _ledsPerAnim;

    private boolean _dirty  = true;

    public Lights(OutliersContainer _container, DriveTrain driveTrain, EndEffector endEffector, OI oi) {
        super(_container);
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT, "rio");
        _config = new CANdleConfiguration();
        // Set LED strip type
        _config.stripType = LEDStripType.RGB;
        // Sets LED brightness
        _config.brightnessScalar = Constants.CANdle.BRIGHTNESS;
        _candle.configAllSettings(_config);
        // set the _color in the constructor to make sure it's not null;
        _slots = 2;
        _colors = new int[_slots][3];
        _ledsPerAnim = Constants.CANdle.NUM_LED / _slots;
        setColor(Constants.CANdle.GREEN, 0);
        switchAnimation(AnimationType.STATIC, 0);
        setColor(Constants.CANdle.RED, 1);
        switchAnimation(AnimationType.STATIC, 1);
    }

    // Set the color of the lights

    public void setColor(int[] color, int slot) {
        if (_colors[slot] != null &&  !colorsMatch(color, _colors[slot]))  {
            _colors[slot] = color;
            _dirty = true;
        }
    }

    public int[] getColor(int slot) {
        return _colors[slot];
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
    public void switchAnimation(AnimationType animation, int slot) {
        if (_currentAnimation == animation && !_dirty) {
            return;
        }
        _currentAnimation = animation;
        switch (animation) {
            case COLOR_FLOW:
                _animate[slot] =
                        new ColorFlowAnimation(
                                _colors[slot][0],
                                _colors[slot][1],
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                ColorFlowAnimation.Direction.Forward,
                                _ledsPerAnim * slot + 8);
                break;
            case FIRE:
                _animate[slot] =
                        new FireAnimation(
                                Constants.CANdle.BRIGHTNESS,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                0.5,
                                0.5,
                                false,
                                _ledsPerAnim * slot + 8);
                break;
            case RAINBOW:
                _animate[slot] =
                        new RainbowAnimation(
                                Constants.CANdle.BRIGHTNESS,
                                Constants.CANdle.SPEED, 
                                _ledsPerAnim,
                                false,
                                _ledsPerAnim * slot + 8);
                break;
            case STROBE:
                _animate[slot] =
                        new StrobeAnimation(
                                _colors[slot][0],
                                _colors[slot][1],
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                _ledsPerAnim * slot + 8);
                break;
            case LARSON:
                _animate[slot] = 
                        new LarsonAnimation(
                                _colors[slot][0], 
                                _colors[slot][1], 
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                BounceMode.Front,
                                7);
                break;
            case RGB_FADE:
                _animate[slot] =
                        new RgbFadeAnimation(
                                Constants.CANdle.BRIGHTNESS,
                                Constants.CANdle.SPEED, 
                                _ledsPerAnim,
                                _ledsPerAnim * slot + 8);
                break;
            case SINGLE_FADE:
                _animate[slot] =
                        new SingleFadeAnimation(
                                _colors[slot][0],
                                _colors[slot][1],
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                _ledsPerAnim * slot + 8);
                break;
            case TWINKLE:
                _animate[slot] =
                        new TwinkleAnimation(
                                _colors[slot][0],
                                _colors[slot][1],
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                Constants.CANdle.TWINKLEPERCENT,
                                _ledsPerAnim * slot + 8);
                break;
            case TWINKLE_OFF:
                _animate[slot] =
                        new TwinkleOffAnimation(
                                _colors[slot][0],
                                _colors[slot][1],
                                _colors[slot][2],
                                0,
                                Constants.CANdle.SPEED,
                                _ledsPerAnim,
                                Constants.CANdle.TWINKLEOFFPERCENT,
                                _ledsPerAnim * slot + 8);
                break;
            case STATIC:
                _animate[slot] = null;
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
        for (int slot = 0; slot < 2; ++slot) {
            if (_animate == null) {
                _candle.clearAnimation(slot); // very important
                // _candle.setLEDs(_colors[0], _colors[1], _colors[2]);
                _candle.setLEDs(
                    _colors[slot][0], 
                    _colors[slot][1], 
                    _colors[slot][2], 
                    0, 
                    _ledsPerAnim * slot + 8,
                    _ledsPerAnim);
            } else {
                _candle.animate(_animate[slot], slot);
            } 
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
