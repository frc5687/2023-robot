/* Team 5687 (C)2022 */
package org.frc5687.lib.math;

public class Vector3d {
    protected double _x;
    protected double _y;
    protected double _z;

    public Vector3d() {
        _x = 0;
        _y = 0;
        _z = 0;
    }

    public Vector3d(double x, double y, double z) {
        _x = x;
        _y = y;
        _z = z;
    }

    public Vector3d(Vector3d other) {
        _x = other._x;
        _y = other._y;
        _z = other._z;
    }

    public double magnitude() {
        return Math.sqrt((_x * _x) + (_y * _y) + (_z * _z));
    }

    public double dot(Vector3d other) {
        return (_x * other._x) + (_y * other._y) + (_z * other._z);
    }

    public Vector3d plus(Vector3d other) {
        return new Vector3d(_x + other._x, _y + other._y, _z + other._z);
    }

    public Vector3d minus(Vector3d other) {
        return new Vector3d(_x - other._x, _y - other._y, _z - other._z);
    }

    public Vector3d scale(double scalar) {
        return new Vector3d(_x * scalar, _y * scalar, _z * scalar);
    }

    public double x() {
        return _x;
    }

    public void setX(double x) {
        _x = x;
    }

    public double y() {
        return _y;
    }

    public void setY(double y) {
        _y = y;
    }

    public double z() {
        return _z;
    }

    public void setZ(double z) {
        _z = z;
    }

    public String toString() {
        return "X: " + _x + ", Y: " + _y + ", Z: " + _z;
    }
}
