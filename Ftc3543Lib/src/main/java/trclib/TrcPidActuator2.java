/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

/**
 * This class is a refactoring of TrcPidActuator.java which allows a TrcPidMotor to be used as the actuator, rather
 * than simply a TrcMotor. The reason for this is that TrcPidMotor allows two TrcMotors to be controlled as a single
 * unit.
 *
 */
public class TrcPidActuator2 extends TrcPidMotor2
{
    private static final String moduleName = "TrcPidActuator2";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private TrcPidMotor2 motor;
    private double minPos = 0.0;
    private double maxPos = 0.0;
    private boolean manualOverride = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the lower limit switch. Required for zero calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     */
    public TrcPidActuator2(
            final String instanceName, TrcPidMotor2 motor, TrcDigitalInput lowerLimitSwitch,
            double minPos,double maxPos)
    {
        super(instanceName,motor.getMotor1(),motor.getMotor2(), motor.getSyncGain(), motor.getController(), motor.getPowerCompensation());

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.motor = motor;
        this.minPos = minPos;
        this.maxPos = maxPos;
        motor.resetPositionOnDigitalInput(lowerLimitSwitch);
        motor.getController().setAbsoluteSetPoint(true);
    }   //TrcPidActuator

    /**
     * This method sets manual override mode. This is useful to override PID control of the actuator in situations
     * where the encoder is not zero calibrated or malfunctioning. Note that this only overrides the encoder but not
     * the limit switch. So if the lower limit switch is engaged, the actuator will not retract even though manual
     * override is true.
     *
     * @param manualOverride specifies true for manual override, false otherwise.
     */
    public void setManualOverride(boolean manualOverride)
    {
        final String funcName = "setManualOverride";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "manualOverrid=%s", Boolean.toString(manualOverride));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.manualOverride = manualOverride;
    }   //setManualOverride

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%s", power);
        }

        if (manualOverride || minPos == 0.0 && maxPos == 0.0)
        {
            super.setPower(power);
        }
        else
        {
            setPowerWithinPosRange(power, minPos, maxPos, true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

}   //class TrcPidActuator
