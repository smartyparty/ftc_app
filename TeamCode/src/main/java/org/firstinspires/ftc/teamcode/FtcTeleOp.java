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
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcUtil;

@TeleOp(name="TeleOp", group="TRCLib")
public class FtcTeleOp extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private static final String moduleName = "FtcTeleOp";

    //Just for testing AutonomousMode functions in TeleOp
    private enum State
    {
        DUMMY
    }   //enum State
    private TrcStateMachine<State> sm = new TrcStateMachine<>(moduleName);
    TrcRobot.RobotCommand cmd;

//    private enum Alliance {
//        RED,
//        BLUE
//    }
    private Alliance alliance = Alliance.RED;

    private enum DriveMode
    {
        TANK_MODE,
        MECANUM_MODE,
    }   //enum DriveMode

//    enum StartPos
//    {
//        NEAR,
//        FAR
//    }

    private StartPos startPosition = StartPos.NEAR;

    protected HalDashboard dashboard;
    protected Robot robot;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private boolean invertedDrive = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

    //Menu-driven values - set the constants to the best values obtained after
    //experimental trials
    double jewelArmExtendedPos = RobotInfo.JEWEL_ARM_EXTENDED;
    double jewelArmRetractedPos = RobotInfo.JEWEL_ARM_RETRACTED;
    double jewelDisplacementDriveTime = RobotInfo.JEWEL_DISPLACEMENT_DRIVE_TIME;
    double jewelDisplacementDrivePower = RobotInfo.JEWEL_DISPLACEMENT_DRIVE_POWER;
    double autoParkPower = RobotInfo.AUTO_PARKING_POWER;

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
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad2, this);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad1, this);
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
//        telemetry.setAutoClear(true);
//        robot.startMode(TrcRobot.RunMode.TELEOP_MODE);
//        telemetry.addLine()
//                .addData("Jewel Init Pos", robot.jewelArm.getPosition());
//
//        telemetry.update();
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

        //Elbow
        double elbowPower = operatorGamepad.getLeftStickY();
        robot.relicArm.elbow1.setPower(elbowPower);
        robot.relicArm.elbow2.setPower(elbowPower);

        dashboard.displayPrintf(2, "xPos=%.2f,yPos=%.2f,heading=%.2f",
                robot.driveBase.getXPosition(), robot.driveBase.getYPosition(),
                robot.driveBase.getHeading());

        dashboard.displayPrintf(3, "jewel=%.2f",
                robot.jewelArm.getPosition());

        dashboard.displayPrintf(4, "d=%.2f,a=%.2f,r=%.2f,g=%.2f,b=%.2f,h=%.2f,s=%.2f,v=%.2f,cn=%f,col=%s",
                robot.jewelDistanceSensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_CM).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.ALPHA).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.RED).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value,
                robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.VALUE).value,
                robot.jewelColorSensor.getRawData( 0, FtcColorSensor.DataType.COLOR_NUMBER).value,
                robot.getObjectColor(robot.jewelColorSensor).toString());

        dashboard.displayPrintf(5, "elbow1=%.1f,elbow2=%.1f",
                robot.relicArm.elbow1.getPower(),
                robot.relicArm.elbow2.getPower());

//        dashboard.displayPrintf(6, "extender=%.1f,flipper=%.1f,rotator=%.1f,claw=%.1f,syringe=%.1f",
//                robot.relicArm.extender.getPosition(),
//                robot.relicArm.claw_flipper.getPosition(),
//                robot.relicArm.claw_rotator.getPosition(),
//                robot.relicArm.claw.getPosition(),
//                robot.relicArm.syringe.getPosition());


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
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed) {

                    }
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
                        robot.jewelColorTrigger.setEnabled(true);
                        robot.jewelArm.setExtended(true);

                        //Wait for color sensor trigger on robot to process data and calculate color
                        sleep(500);

                        //Simulate 10 tries
                        Robot.ObjectColor jewelColor = robot.getObjectColor(robot.jewelColorSensor);

                        int retries = 0;
                        while (retries < 10 && jewelColor == Robot.ObjectColor.NO ) {
                            jewelColor = robot.getObjectColor(robot.jewelColorSensor);
                            retries++;
                        }
//
////                        telemetry.clear();
////
////                        telemetry.addLine()
////                                .addData("Distance (cm)", robot.jewelDistanceSensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_CM).value)
////                                .addData("Color", jewelColor.toString())
////                                .addData("Alpha", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.ALPHA).value)
////                                .addData("Red  ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.RED).value)
////                                .addData("Green", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value)
////                                .addData("Blue ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value)
////                                .addData("Hue ", robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value)
////                                .addData("Color #", robot.jewelColorSensor.getRawData( 0, FtcColorSensor.DataType.COLOR_NUMBER).value.intValue());
////
////                        telemetry.update();
//
                        robot.jewelColorTrigger.setEnabled(false);
//
//                        //Retract arm
//                        robot.jewelArm.setExtended(false);
//
//                        sleep(500);
//
//                        if (jewelColor != Robot.ObjectColor.NO ) {
//                            //displaceJewel(jewelColor); //moved this to SharedFunctions.java
//                            SharedFunctions.displaceJewel(robot, jewelColor, alliance, jewelDisplacementDrivePower, jewelDisplacementDriveTime);
//                        }
//
//                        //driveToSafeZone();
//                        SharedFunctions.driveToSafeZone(robot, alliance, startPosition, autoParkPower);
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed) {
                        robot.jewelArm.setExtended(false);
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

    private void waitForJewelArm(double pos) {
        while (robot.jewelArm.getPosition() != pos) {
            dashboard.displayPrintf(3, "jewel=%.2f",
                    robot.jewelArm.getPosition());
            idle();
        }
    }

    private void driveToSafeZone() {
        double driveTime = 0.0;
        double xpower = 0.0;
        double ypower = 0.0;
        double xSign = (alliance == Alliance.RED ? 1.0 : -1.0);
        double ySign = (alliance == Alliance.RED ? 1.0 : -1.0);

        //Blue advances to safezone forward (toward from color sensor / negative sign)
        //Red advances to safezone rearward (away from color sensor / positive sign)

        //Blue arena center = -x
        //Red arena center = +x

        switch (startPosition) {
            case NEAR:
                //drive forward
                ypower = autoParkPower * ySign;
                driveTime = 1;
                driveForTime(xpower,ypower,0, driveTime);
                break;
            case FAR:
                //drive sideways
                ypower = 0;
                xpower = autoParkPower * xSign;
                driveTime = 0.4;
                driveForTime(xpower, ypower, 0, driveTime);

                //drive forward
                driveTime = 1.0;
                xpower = 0;
                ypower = autoParkPower * ySign;
                driveForTime(xpower, ypower, 0, driveTime);
                break;
        }
    }

    private void displaceJewel(Robot.ObjectColor jewelColor) {
        //Absolute drive distance required to displace a jewel
        //final double displacement_distance = 6;
        double xDrivePower = 0.0;
        double yDrivePower = 0.0;
        double turnPower = 0.0;
        double directionFactor = (alliance == Alliance.BLUE ? -1.0 : 1.0);
//        double targetX = 0.0;
//        double targetY = 0.0;

        //Drive at half-power
        //robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
        //No movement along x-axis
        //targetX = 0.0;
        //No gyro heading
        //robot.targetHeading = 0.0;

        //ASSUMPTION: jewel color sensor faces forward
        //Determine which direction to move to displace the correct jewel by specifying
        //a Y-axis drive target value of  + or - the desired distance
        if (jewelColor == Robot.ObjectColor.RED && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.BLUE) {
            //drive backward
            //targetY = -displacement_distance;

            yDrivePower = jewelDisplacementDrivePower;
        } else if (jewelColor == Robot.ObjectColor.BLUE && alliance == Alliance.RED ||
                jewelColor == Robot.ObjectColor.RED && alliance == Alliance.BLUE) {
            //drive forward
            //targetY = displacement_distance;
            yDrivePower = -jewelDisplacementDrivePower;
        }

        driveForTime(xDrivePower, yDrivePower, turnPower, jewelDisplacementDriveTime);
    }

    private void driveForTime(double xpower, double ypower, double turnpower, double time) {
        long start = TrcUtil.getCurrentTimeMillis();
        long expired = 0;
        while (expired < (time * 1000)) {
            robot.driveBase.mecanumDrive_Cartesian(xpower, ypower, turnpower);
            expired = TrcUtil.getCurrentTimeMillis() - start;
        }

        robot.driveBase.stop();
    }

    private void showMenu() {
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, robot);

        FtcChoiceMenu<StartPos> startPositionMenu = new FtcChoiceMenu<>("Start Pos:", allianceMenu, robot);

        allianceMenu.addChoice("Red", Alliance.RED, true, startPositionMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPositionMenu);

        FtcValueMenu jewelExtPosMenu = new FtcValueMenu(
                "Jewel Ext Pos:", startPositionMenu, robot,
                0.0, 1.0, 0.05, RobotInfo.JEWEL_ARM_EXTENDED, " %.2f");

        FtcValueMenu jewelRetPosMenu = new FtcValueMenu(
                "Jewel Ret Pos:", jewelExtPosMenu, robot,
                -1, 1.0, 0.05, RobotInfo.JEWEL_ARM_RETRACTED, " %.2f");


        startPositionMenu.addChoice("Near", StartPos.NEAR, true, jewelExtPosMenu);
        startPositionMenu.addChoice("Far", StartPos.FAR, false, jewelExtPosMenu);

        jewelExtPosMenu.setChildMenu(jewelRetPosMenu);

        FtcValueMenu jewelDispDriveTimeMenu = new FtcValueMenu(
                "Jewel Disp Time:", jewelExtPosMenu, robot,
                0.0, 1.5, 0.1, RobotInfo.JEWEL_DISPLACEMENT_DRIVE_TIME, " %.2f");

        jewelRetPosMenu.setChildMenu(jewelDispDriveTimeMenu);

        FtcValueMenu jewelDispDrivePowerMenu = new FtcValueMenu(
                "Jewel Disp Pwr:", jewelDispDriveTimeMenu, robot,
                0, 1, 0.1, RobotInfo.JEWEL_DISPLACEMENT_DRIVE_POWER, " %.2f");

        jewelDispDriveTimeMenu.setChildMenu(jewelDispDrivePowerMenu);

        FtcValueMenu autoParkPowerMenu = new FtcValueMenu(
                "Park Pwr:", jewelDispDrivePowerMenu, robot,
                0, 1, 0.1, RobotInfo.AUTO_PARKING_POWER, " %.2f");

        jewelDispDrivePowerMenu.setChildMenu(autoParkPowerMenu);

        FtcMenu.walkMenuTree(allianceMenu, this);

        jewelArmExtendedPos = jewelExtPosMenu.getCurrentValue();
        jewelArmRetractedPos = jewelRetPosMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();
        startPosition = startPositionMenu.getCurrentChoiceObject();
        jewelDisplacementDriveTime = jewelDispDriveTimeMenu.getCurrentValue();
        jewelDisplacementDrivePower = jewelDispDrivePowerMenu.getCurrentValue();
        autoParkPower = autoParkPowerMenu.getCurrentValue();
    }
}   //class FtcTeleOp