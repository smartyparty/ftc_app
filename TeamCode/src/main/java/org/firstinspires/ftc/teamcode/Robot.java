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

package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftclib.FtcAnalogGyro;
import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcColorSensor;
import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcUtil;

public class Robot implements TrcPidController.PidInput, FtcMenu.MenuButtons, TrcAnalogTrigger.TriggerHandler
{
    private static final boolean USE_IMU = true;
    private static final boolean USE_ANALOG_GYRO = false;
    private static final boolean USE_SPEECH = false;
    private static final boolean USE_VUFORIA = false;
    private static final boolean USE_JEWEL_COLOR_SENSOR = false;
    private static final boolean USE_CRYPTO_COLOR_SENSOR = false;
    private static final boolean USE_ANALOG_TRIGGERS = true;

    private static final String moduleName = "Robot";

    enum ObjectColor
    {
        NO,
        RED,
        BLUE
    }

    //
    // Global objects.
    //
    FtcOpMode opMode;
    HalDashboard dashboard;
    TrcDbgTrace tracer;
    FtcRobotBattery battery = null;
    FtcAndroidTone androidTone;
    TextToSpeech textToSpeech = null;
    //
    // Sensors.
    //
    FtcBNO055Imu imu = null;
    TrcGyro gyro = null;
    double targetHeading = 0.0;

    static final double[] colorTriggerPoints = {
            RobotInfo.RED1_LOW_THRESHOLD, RobotInfo.RED1_HIGH_THRESHOLD,
            RobotInfo.BLUE_LOW_THRESHOLD, RobotInfo.BLUE_HIGH_THRESHOLD,
            RobotInfo.RED2_LOW_THRESHOLD, RobotInfo.RED2_HIGH_THRESHOLD};

    FtcColorSensor jewelColorSensor = null;
    TrcAnalogTrigger<FtcColorSensor.DataType> jewelColorTrigger = null;

    FtcColorSensor cryptoColorSensor = null;
    TrcAnalogTrigger<FtcColorSensor.DataType> cryptoColorTrigger = null;
    int redCryptoBarCount = 0;
    int blueCryptoBarCount = 0;

    //
    // Vision subsystems.
    //
    VuforiaVision vuforiaVision = null;
    RelicRecoveryVuMark prevVuMark = null;

    //
    // DriveBase subsystem.
    //
    FtcDcMotor leftFrontWheel = null;
    FtcDcMotor rightFrontWheel = null;
    FtcDcMotor leftRearWheel = null;
    FtcDcMotor rightRearWheel = null;
    TrcDriveBase driveBase = null;

    TrcPidController encoderXPidCtrl = null;
    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidDrive pidDrive = null;

    TrcPidController visionPidCtrl = null;
    TrcPidDrive visionDrive = null;

    //
    // Other subsystems.
    //


    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        tracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                (TextView)((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        battery = new FtcRobotBattery();
        androidTone = new FtcAndroidTone("AndroidTone");
        if (USE_SPEECH)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            textToSpeech.speak("Init Starting", TextToSpeech.QUEUE_FLUSH, null);
        }
        //
        // Initialize sensors.
        //
        if (USE_IMU)
        {
            imu = new FtcBNO055Imu("imu");
            gyro = imu.gyro;
        }
        else if (USE_ANALOG_GYRO)
        {
            gyro = new FtcAnalogGyro("analogGyro", RobotInfo.ANALOG_GYRO_VOLT_PER_DEG_PER_SEC);
            ((FtcAnalogGyro)gyro).calibrate();
            gyro.setScale(0, RobotInfo.ANALOG_GYRO_SCALE);
            //
            // Wait for gyro calibration to complete if not already.
            //
            while (gyro.isCalibrating())
            {
                TrcUtil.sleep(10);
            }
        }

        if (USE_JEWEL_COLOR_SENSOR)
        {
            jewelColorSensor = new FtcColorSensor("jewelColorRangeSensor");
            if (USE_ANALOG_TRIGGERS)
            {
                jewelColorTrigger = new TrcAnalogTrigger(
                        "jewelColorTrigger", jewelColorSensor, 0, FtcColorSensor.DataType.HUE,
                        colorTriggerPoints, this);
            }
        }

        if (USE_CRYPTO_COLOR_SENSOR)
        {
            cryptoColorSensor = new FtcColorSensor("cryptoColorRangeSensor");
            if (USE_ANALOG_TRIGGERS)
            {
                cryptoColorTrigger = new TrcAnalogTrigger(
                        "cryptoColorTrigger", cryptoColorSensor, 0, FtcColorSensor.DataType.HUE,
                        colorTriggerPoints, this);
            }
        }

        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            vuforiaVision = new VuforiaVision(this, cameraViewId);
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("left_front_drive");
        rightFrontWheel = new FtcDcMotor("right_front_drive");
        leftRearWheel = new FtcDcMotor("left_rear_drive");
        rightRearWheel = new FtcDcMotor("right_rear_drive");

        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD),
                RobotInfo.ENCODER_X_TOLERANCE, this);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, this);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, this);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        visionPidCtrl = new TrcPidController(
                "visionPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.VISION_KP, RobotInfo.VISION_KI, RobotInfo.VISION_KD),
                RobotInfo.VISION_TOLERANCE, this);
        visionPidCtrl.setAbsoluteSetPoint(true);

        visionDrive = new TrcPidDrive("visionDrive", driveBase, null, visionPidCtrl, null);
        visionDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        visionDrive.setBeep(androidTone);

        //
        // Initialize other subsystems.
        //
        //TODO

        //
        // Tell the driver initialization is complete.
        //
        if (textToSpeech != null)
        {
            textToSpeech.speak("Init complete!", TextToSpeech.QUEUE_ADD, null);
        }
    }   //Robot

    void startMode(TrcRobot.RunMode runMode)
    {
        //
        // Since our gyro is analog, we need to enable its integrator.
        //
        gyro.setEnabled(true);

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }

        //
        // Reset all X, Y and heading values, including motor encoders
        //
        driveBase.resetPosition();
        targetHeading = 0.0;

    }   //startMode

    void stopMode(TrcRobot.RunMode runMode)
    {
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] %17s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading,
                battery.getVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    ObjectColor getObjectColor(FtcColorSensor sensor)
    {
        ObjectColor color = ObjectColor.NO;

        if (sensor != null)
        {
            double hue = sensor.getRawData(0, FtcColorSensor.DataType.HUE).value;
            double sat = sensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value;
            double value = sensor.getRawData(0, FtcColorSensor.DataType.VALUE).value;

            if (sat > 0.0 && value > 0.0)
            {
                if (hue >= RobotInfo.RED1_LOW_THRESHOLD && hue <= RobotInfo.RED1_HIGH_THRESHOLD ||
                        hue >= RobotInfo.RED2_LOW_THRESHOLD && hue >= RobotInfo.RED2_HIGH_THRESHOLD)
                {
                    color = ObjectColor.RED;
                }
                else if (hue >= RobotInfo.BLUE_LOW_THRESHOLD && hue <= RobotInfo.BLUE_HIGH_THRESHOLD)
                {
                    color = ObjectColor.BLUE;
                }
            }
        }

        return color;
    }   //getObjectColor

    double getObjectHsvHue(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getRawData(0, FtcColorSensor.DataType.HUE).value;
        }

        return value;
    }   //getObjectHsvHue

    double getObjectHsvSaturation(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value;
        }

        return value;
    }   //getObjectHsvSaturation

    double getObjectHsvValue(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getRawData(0, FtcColorSensor.DataType.VALUE).value;
        }

        return value;
    }   //getObjectHsvValue

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            input = driveBase.getXPosition();
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroPidCtrl)
        {
            input = driveBase.getHeading();
        }
        else if (pidCtrl == visionPidCtrl)
        {
            RelicRecoveryVuMark vuMark = vuforiaVision.getVuMark();

            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                input = vuforiaVision.getVuMarkPosition().get(0)/RobotInfo.MM_PER_INCH;
            }

            if (textToSpeech != null && vuMark != prevVuMark)
            {
                String sentence = null;

                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                    sentence = String.format("%s is %s.", vuMark.toString(), "in view");
                }
                else if (prevVuMark != null)
                {
                    sentence = String.format("%s is %s.", prevVuMark.toString(), "out of view");
                }

                if (sentence != null)
                {
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler interface.
    //

    @Override
    public void triggerEvent(TrcAnalogTrigger<?> analogTrigger, int zoneIndex, double zoneValue)
    {
        FtcColorSensor colorSensor = analogTrigger == jewelColorTrigger? jewelColorSensor: cryptoColorSensor;
        ObjectColor color = getObjectColor(colorSensor);

        if (analogTrigger == cryptoColorTrigger)
        {
            if (color == ObjectColor.RED)
            {
                redCryptoBarCount++;
                if (textToSpeech != null)
                {
                    textToSpeech.speak(
                            String.format("%s red crypto.", redCryptoBarCount), TextToSpeech.QUEUE_FLUSH, null);
                }
            }
            else if (color == ObjectColor.BLUE)
            {
                blueCryptoBarCount++;
                if (textToSpeech != null)
                {
                    textToSpeech.speak(
                            String.format("%s blue crypto.", blueCryptoBarCount), TextToSpeech.QUEUE_FLUSH, null);
                }
            }
        }
        else if (textToSpeech != null)
        {
            textToSpeech.speak(
                    String.format("%s jewel found.", color.toString()), TextToSpeech.QUEUE_FLUSH, null);
        }
    }   // triggerEvent

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return opMode.gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return opMode.gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton

//    private void setDrivePID(double xDistance, double yDistance, double heading)
//    {
//        double degrees = Math.abs(heading - driveBase.getHeading());
//        xDistance = Math.abs(xDistance);
//        yDistance = Math.abs(yDistance);
//        //
//        // No oscillation if turn-only.
//        //
//        boolean noOscillation = degrees != 0.0 && xDistance == 0.0 && yDistance == 0.0;
//        gyroPidCtrl.setNoOscillation(noOscillation);
//        tracer.traceInfo("setDrivePID", "NoOscillation=%s", Boolean.toString(noOscillation));
//        if (xDistance != 0.0 && xDistance < RobotInfo.SMALL_X_THRESHOLD)
//        {
//            //
//            // Small X movement, use stronger X PID to overcome friction.
//            //
//            encoderXPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.ENCODER_SMALL_X_KP, RobotInfo.ENCODER_SMALL_X_KI, RobotInfo.ENCODER_SMALL_X_KD));
//        }
//        else
//        {
//            //
//            // Use normal X PID.
//            //
//            encoderXPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD));
//        }
//
//        if (yDistance != 0.0 && yDistance < RobotInfo.SMALL_Y_THRESHOLD)
//        {
//            //
//            // Small Y movement, use stronger Y PID to overcome friction.
//            //
//            encoderYPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.ENCODER_SMALL_Y_KP, RobotInfo.ENCODER_SMALL_Y_KI, RobotInfo.ENCODER_SMALL_Y_KD));
//        }
//        else
//        {
//            //
//            // Use normal Y PID.
//            //
//            encoderYPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD));
//        }
//
//        if (degrees != 0.0 && degrees < RobotInfo.SMALL_TURN_THRESHOLD)
//        {
//            //
//            // Small turn, use stronger turn PID to overcome friction.
//            //
//            gyroPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.GYRO_SMALL_TURN_KP, RobotInfo.GYRO_SMALL_TURN_KI, RobotInfo.GYRO_SMALL_TURN_KD));
//        }
//        else
//        {
//            //
//            // Use normal Y PID.
//            //
//            gyroPidCtrl.setPidCoefficients(
//                    new TrcPidController.PidCoefficients(
//                            RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD));
//        }
//    }   //setDrivePID
//
//    void setPIDDriveTarget(
//            double xDistance, double yDistance, double heading, boolean holdTarget, TrcEvent event)
//    {
//        setDrivePID(xDistance, yDistance, heading);
//        pidDrive.setTarget(xDistance, yDistance, heading, holdTarget, event);
//    }   //setPIDDriveTarget

}   //class Robot