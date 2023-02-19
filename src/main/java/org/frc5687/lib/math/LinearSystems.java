package org.frc5687.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class LinearSystems {

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    public static LinearSystem<N3, N2, N3> createDifferentialSwerveModule(
            DCMotor motor, double Js, double Jw, double Gs, double Gw) {

        var Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
        var Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, Vs, Vw, -Vw);
        var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N3(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }

    /**
     * Creates a StateSpace model of a differential swerve module with current input.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    public static LinearSystem<N3, N2, N3> createDifferentialSwerveModuleCurrent(
            DCMotor motor, double Js, double Jw, double Gs, double Gw, double Fs, double Fw) {

        // friction damping
        var fS = -Fs / Js;
        var fW = -Fw / Jw;
        // current steer
        var Cs = Gs * motor.KtNMPerAmp / Js;
        var Cw = Gw * motor.KtNMPerAmp / Jw;

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(
                                0.0, 1.0, 0.0,
                                0.0, fS, 0.0,
                                0.0, 0.0, fW);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(
                0.0, 0.0,
                Cs, Cs,
                Cw, -Cw);
        var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N3(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }
    public static DCMotor getFalcon500FOC(int numMotors) {
        return new DCMotor(
                12,
                5.84,
                304,
                1.5,
                Units.rotationsPerMinuteToRadiansPerSecond(6080.0),
                numMotors
        );
    }
}