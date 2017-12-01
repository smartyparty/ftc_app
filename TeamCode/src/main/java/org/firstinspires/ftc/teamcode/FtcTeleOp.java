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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcColorSensor;
import ftclib.FtcDistanceSensor;
import ftclib.FtcGamepad;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcGameController;
import trclib.TrcRobot;
import trclib.TrcTimer;

@TeleOp(name="TeleOp", group="TRCLib")
public class FtcTeleOp extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private static final String moduleName = "FtcTeleOp";

    private enum Alliance {
        RED,
        BLUE
    }

    private Alliance alliance = Alliance.RED;

    private enum DriveMode
    {
        TANK_MODE,
        MECANUM_MODE,
    }   //enum DriveMode

    protected HalDashboard dashboard;
    protected Robot robot;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private boolean invertedDrive = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

    //Menu-driven values
    double jewelArmExtendedPos = RobotInfo.JEWEL_ARM_EXTENDED;
    double jewelArmRetractedPos = RobotInfo.JEWEL_ARM_RETRACTED;

    //For wait/events
    private TrcEvent event = new TrcEvent(moduleName);
    private TrcTimer timer = new TrcTimer(moduleName);

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.TELEOP_MODE);

        dashboard = robot.dashboard;

        //Show configuration menu for this OpMode
        showMenu();

        //Set robot config values from menu
        robot.jewelArm.setExtendedPos(jewelArmExtendedPos);
        robot.jewelArm.setRetractedPos(jewelArmRetractedPos);

        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        telemetry.setAutoClear(true);
        robot.startMode(TrcRobot.RunMode.TELEOP_MODE);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.TELEOP_MODE);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        switch(driveMode)
        {
            case TANK_MODE:
                double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                        leftPower, rightPower, Boolean.toString(invertedDrive));
                break;

            case MECANUM_MODE:
                double x = driverGamepad.getLeftStickX(true)*drivePowerScale;
                double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                double rot = (driverGamepad.getRightTrigger(true) -
                        driverGamepad.getLeftTrigger(true))*drivePowerScale;
                robot.driveBase.mecanumDrive_Cartesian(x, y, rot, invertedDrive);
                dashboard.displayPrintf(1, "Mecanum:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                        x, y, rot, Boolean.toString(invertedDrive));
                break;
        }
        dashboard.displayPrintf(2, "xPos=%.2f,yPos=%.2f,heading=%.2f",
                robot.driveBase.getXPosition(), robot.driveBase.getYPosition(),
                robot.driveBase.getHeading());

        // send the info back to driver station using telemetry function.
//        telemetry.clear();
//        telemetry.addLine()
//                .addData("Distance (cm)", robot.jewelDistanceSensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_CM).value)
//                .addData("Alpha", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.ALPHA).value)
//                .addData("Red  ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.RED).value)
//                .addData("Green", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value)
//                .addData("Blue ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value)
//                .addData("Color #", robot.jewelColorSensor.getRawData( 0, FtcColorSensor.DataType.COLOR_NUMBER).value);
//        telemetry.update();

    }   //runPeriodic


    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                        driveMode = DriveMode.MECANUM_MODE;
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        driveMode = DriveMode.TANK_MODE;
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed) {
                        robot.jewelColorTrigger.setEnabled(true);
                        robot.jewelArm.setExtended(true);

                        sleep(500);
                        Robot.ObjectColor jewelColor = robot.getObjectColor(robot.jewelColorSensor);

                        telemetry.clear();

                        telemetry.addLine()
                                .addData("Distance (cm)", robot.jewelDistanceSensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_CM).value)
                                .addData("Color", jewelColor.toString())
                                .addData("Alpha", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.ALPHA).value)
                                .addData("Red  ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.RED).value)
                                .addData("Green", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value)
                                .addData("Blue ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value)
                                .addData("Hue ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value)
                                .addData("Color #", robot.jewelColorSensor.getRawData( 0, FtcColorSensor.DataType.COLOR_NUMBER).value.intValue());

                        telemetry.update();

                        robot.jewelColorTrigger.setEnabled(false);

                        displaceJewel(jewelColor);
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                        robot.jewelArm.setExtended(false);
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 0.5: 1.0;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    //robot.relicArm.elbow.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:

                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {

                    }
                    else
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {

                    }
                    else
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (pressed)
                    {

                    }
                    break;

                case FtcGamepad.GAMEPAD_START:
                    if (pressed)
                    {

                    }
                    break;
            }
        }
    }   //buttonEvent

    private void displaceJewel(Robot.ObjectColor jewelColor) {
        //Absolute drive distance required to displace a jewel
        final double displacement_distance = 6;
        double targetX = 0.0;
        double targetY = 0.0;

        //Drive at half-power
        robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
        //No movement along x-axis
        targetX = 0.0;
        //No gyro heading
        robot.targetHeading = 0.0;

        //We assume that jewel color sensor faces fowards

        //Determine which direction to move to displace the correct jewel by specifying
        //a Y-axis drive target value of  + or - the desired distance
        if (jewelColor == Robot.ObjectColor.RED && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.BLUE) {
            //drive backwards
            targetY = -displacement_distance;
        } else if (jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.RED && alliance == Alliance.BLUE) {
            //drive forwards
            targetY = displacement_distance;
        }

        robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 2.0);
    }
    private void showMenu() {
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, robot);
        allianceMenu.addChoice("Red", Alliance.RED, true, null);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, null);

        FtcValueMenu jewelExtPosMenu = new FtcValueMenu(
                "Jewel Extended Position:", allianceMenu, robot,
                0.0, 1.0, 0.1, 0.5, " %.2f");

        FtcValueMenu jewelRetPosMenu = new FtcValueMenu(
                "Jewel Retracted Position:", allianceMenu, robot,
                -1.0, 0, 0.1, -0.5, " %.2f");

        FtcMenu.walkMenuTree(allianceMenu, this);

        jewelArmExtendedPos = jewelExtPosMenu.getCurrentValue();
        jewelArmRetractedPos = jewelRetPosMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();
    }
}   //class FtcTeleOp