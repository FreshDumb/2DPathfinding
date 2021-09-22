using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathUtils
{
    /// Utils
    /// 
    public static float[] SolveQuadraticFormula(float _a, float _b, float _c)
    {
        float[] result = new float[2];
        float determinant = _b * _b - 4 * _a * _c;
        if (determinant < 0)
        {
            return null;
        }
        float determinantSqrt = Mathf.Sqrt(determinant);

        result[0] = (-_b + determinantSqrt) / (2 * _a);
        result[1] = (-_b - determinantSqrt) / (2 * _a);
        return result;
    }
    public static float[] InAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        return SolveQuadraticFormula(tempA, tempB, tempC);
    }
    public static float MaxInAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        float[] tempResult = SolveQuadraticFormula(tempA, tempB, tempC);
        if (tempResult == null)
        {
            return float.MinValue;
        }
        else
        {
            return Mathf.Max(tempResult[0], tempResult[1]);
        }
    }
    public static float MinInAirTime(float _startingY, float _targetY, float _yVelocity, float _gravity)
    {
        float tempC = _startingY - _targetY;
        float tempB = _yVelocity;
        float tempA = -0.5f * _gravity;

        float[] tempResult = SolveQuadraticFormula(tempA, tempB, tempC);
        if (tempResult == null)
        {
            return float.MinValue;
        }
        else
        {
            return Mathf.Min(tempResult[0], tempResult[1]);
        }
    }
    public static float GetPreciseJumpStartVelocity(float _startingY, float _targetY, float _jumpTime, float _gravity)
    {
        return (_targetY - _startingY + 0.5f * _gravity * _jumpTime * _jumpTime) / _jumpTime;
    }
    public static Vector2 GetCurrentPositionInJump(float _currentTime, float _startingX, float _startingY, float _ystartVelocity, float _xVelocity, float _gravity)
    {
        float tempX = _startingX + _currentTime * _xVelocity;
        float tempY = _startingY + _ystartVelocity * _currentTime - 0.5f * _gravity * _currentTime * _currentTime;
        return new Vector2(tempX, tempY);
    }
}
