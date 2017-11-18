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

package ftclib;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import trclib.TrcDbgTrace;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements a generic Distance sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcDistanceSensor extends TrcSensor<FtcDistanceSensor.DataType>
{
    private static final String moduleName = "FtcDistanceSensor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum DataType
    {
        DISTANCE_MM,
        DISTANCE_CM,
        DISTANCE_METER,
        DISTANCE_INCH
    }   //enum DataType

    public DistanceSensor sensor;
    private double distanceMmData = 0;
    private long distanceMmTagId = -1;
    private double distanceCmData = 0;
    private long distanceCmTagId = -1;
    private double distanceMeterData = 0;
    private long distanceMeterTagId = -1;
    private double distanceInchData = 0;
    private long distanceInchTagId = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcDistanceSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        sensor = hardwareMap.get(DistanceSensor.class, instanceName);
    }   //FtcDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDistanceSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcDistanceSensor

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified index and type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData<Double> data = null;
        long currTagId = FtcOpMode.getLoopCounter();

        switch (dataType)
        {
            case DISTANCE_MM:
                if (currTagId != distanceMmTagId)
                {
                    distanceMmData = sensor.getDistance(DistanceUnit.MM);
                    distanceMmTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), distanceMmData);
                break;

            case DISTANCE_CM:
                if (currTagId != distanceCmTagId)
                {
                    distanceCmData = sensor.getDistance(DistanceUnit.CM);
                    distanceCmTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), distanceCmData);
                break;

            case DISTANCE_METER:
                if (currTagId != distanceMeterTagId)
                {
                    distanceMeterData = sensor.getDistance(DistanceUnit.METER);
                    distanceMeterTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), distanceMeterData);
                break;

            case DISTANCE_INCH:
                if (currTagId != distanceInchTagId)
                {
                    distanceInchData = sensor.getDistance(DistanceUnit.INCH);
                    distanceInchTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), distanceInchData);
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%d)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcDistanceSensor
