/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Main Autonomous OpMode")
public class MainAutonomousOpMode extends LinearOpMode {
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaLocalizer.Parameters cameraParameters = null;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final int CYCLE_MS = 50;
    private LocalDateTime stopTime = LocalDateTime.MIN;
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    UltrasonicSensor usSensor;
    Gamepad gamepad;
    private static final String VUFORIA_KEY = "AX3wJCr/////AAABmXqqKIuTJkMatbvyrDkSphp5h9VYO42DWDiKLucp30xEvP98y9OIRKGNszB+EpBJ4cPqww5PWGd6BPOl7kHDKajvlJClovU1+L4gNxeM0pvROvjulRueLD7JCqzM7yWina4gLO1YTeWIUGqF1v1Qh34137m65frOnjbPcBgQk2O4ky740K3T/SM541xP4ALr3FSvyl2xQD1EBu2xI49XL2bLtsztTuUUUmyS07lvmYNt4kH1+108Bqvca+GYnufjYs8mlDG4qYSF6UhZyGHhjskblJWKaNQT2Lph3JgxMXfcaV40/qkq9C52GVbIk/QLEDRnKq3Ezce13ZM+GK+YItEAYXJ+hUZHTUIbVedF8uZ4";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private static final float linearSpeedPerPower = 1.0f;
    private static final float rotationSpeedPerPower = 1.0f;
    private static final float millimetersPerTableRow = 10.0f;
    private static final float ultrasoundThreshold = 1.0f;
    private VuforiaLocalizer vuforia = null;
    private static final float[][] trackableTransforms = {
            { halfField, quadField, mmTargetHeight, 90, 0, -90 },
            { halfField, -quadField, mmTargetHeight, 90, 0, -90 },
            { 0, -halfField, mmTargetHeight, 90, 0, 180 },
            { 0, halfField, mmTargetHeight, 90, 0, 0 },
            { -halfField, 0, mmTargetHeight, 90, 0, 90 }
    };
    private static final String[] trackableNames = {
            "Blue Tower Goal Target",
            "Red Tower Goal Target",
            "Red Alliance Target",
            "Blue Alliance Target",
            "Front Wall Target"
    };
    private static final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;
    private static final float CAMERA_LEFT_DISPLACEMENT     = 0;
    WebcamName webcamName = null;
    VectorF translation = null;
    Orientation rotation = null;
    VuforiaTrackables targetsUltimateGoal = null;
    ArrayList<RobotMoveItem> moveQueue = new ArrayList<RobotMoveItem>();
    private static final double backupDistance = 200;
    private static double targetX = 0;
    private static double targetY = 0;
    private static double targetRotation = 0;
    @Override public void runOpMode() {
        initializeProgram();
        runProgram();
        shutdownProgram();
    }
    private void runProgram() {
        while (shouldContinue()) mainLoop();
    }
    private void mainLoop() {
        updateLocations();
        moveRobot();
        finalizeLoopStage();
    }
    private void initializeProgram() {
        initializeVuforia();
        initializeTargets();
        initializeMotors();
        initializeSensors();
        beginMainProgram();
    }
    private void moveRobot() {
        if(!backupIfNeeded() && LocalDateTime.now().isAfter(stopTime)) {
            planMotion();
        }
    }
    private void generateMoveQueue() {
        ArrayList<RobotMoveItem> initialQueue = constructMovesFromTable(generateTable(), (int)(translation.get(0) / millimetersPerTableRow), (int)(translation.get(1) / millimetersPerTableRow), (int)(rotation.toAxesOrder(XYZ).toAngleUnit(DEGREES).thirdAngle / 90.0), (int)(targetX / millimetersPerTableRow), (int)(targetY / millimetersPerTableRow), (int)(targetRotation / (Math.PI / 2)));
        initialQueue.add(0, new RobotMoveItem(true, (int)(rotation.toAxesOrder(XYZ).toAngleUnit(DEGREES).thirdAngle / 90.0) * 90.0 - rotation.toAxesOrder(XYZ).toAngleUnit(DEGREES).thirdAngle));
        initialQueue.add(new RobotMoveItem(true, (int)(targetRotation / (Math.PI / 2)) * (Math.PI / 2) - (int)(targetRotation / (Math.PI / 2))));
        moveQueue = initialQueue;
    }
    private void setTarget() {

    }
    private boolean[][] generateTable() {
        double fullField = halfField * 2;
        int dimension = (int)(fullField / millimetersPerTableRow);
        boolean[][] outputTable = new boolean[dimension][dimension];
        int xCounter = 0;
        int yCounter = 0;
        for(double xValue = 0; xValue <= fullField; xValue += millimetersPerTableRow) {
            for(double yValue = 0; yValue <= fullField; yValue += millimetersPerTableRow) {
                outputTable[xCounter][yCounter] = getTableEntryAtLocation(xValue, yValue);
                yCounter++;
            }
            xCounter++;
        }
        return outputTable;
    }
    private boolean getTableEntryAtLocation(double xPosition, double yPosition) {
        return false;
    }
    private void backupRobot() {
        moveLinear(-backupDistance);
        moveQueue.clear();
        generateMoveQueue();
    }
    private boolean backupIfNeeded() {
        if(checkTooClose()) {
            backupRobot();
            return true;
        }
        return false;
    }
    private void initializeSensors() {
        usSensor = hardwareMap.get(UltrasonicSensor.class, "Ultrasonic 1");
        gamepad = hardwareMap.get(Gamepad.class, "Gamepad 1");
    }
    private void planMotion() {
        if(!moveQueue.isEmpty()) {
            actOnMoveItem(moveQueue.get(0));
            moveQueue.remove(0);
        } else {

        }
    }
    private boolean shouldContinue() {
        return !isStopRequested() && opModeIsActive();
    }
    private void initializeVuforia() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        cameraParameters.vuforiaLicenseKey = VUFORIA_KEY;
        cameraParameters.cameraName = webcamName;
        cameraParameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(cameraParameters);
    }
    private void initializeTargets() {
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables.addAll(targetsUltimateGoal);
        for(int i = 0; i < trackableNames.length; i++) {
            VuforiaTrackable trackable = allTrackables.get(i);
            trackable.setName(trackableNames[i]);
            transform(trackable, trackableTransforms[i]);
        }
    }
    private boolean checkTooClose() {
        return usSensor.getUltrasonicLevel() < ultrasoundThreshold;
    }
    private int[] constructDjikstraRoute(int graphSize, int[][] edges, double[][] weights, int start, int end) {
        double[] distances = new double[graphSize];
        int[] routeFrom = new int[graphSize];
        for(int i = 0; i < graphSize; i++) {
            distances[i] = Double.MAX_VALUE;
            if(i == start) distances[i] = 0;
            routeFrom[i] = -1;
        }
        PriorityQueue<DjikstraPriorityQueueItem> queue = new PriorityQueue<>(new DjikstraPriorityQueueComparator());
        queue.add(new DjikstraPriorityQueueItem(start, 0));
        while(!queue.isEmpty()) {
            DjikstraPriorityQueueItem queueItem = queue.remove();
            for(int i = 0; i < edges[queueItem.index].length; i++) {
                if(queueItem.distance + weights[queueItem.index][i] < distances[edges[queueItem.index][i]]) {
                    distances[edges[queueItem.index][i]] = queueItem.distance + weights[queueItem.index][i];
                    queue.add(new DjikstraPriorityQueueItem(edges[queueItem.index][i], distances[edges[queueItem.index][i]]));
                    routeFrom[edges[queueItem.index][i]] = queueItem.index;
                }
            }
        }
        if(routeFrom[end] == -1) return new int[0];
        ArrayList<Integer> output = new ArrayList<>();
        int currentIndex = end;
        while(currentIndex != -1) {
            output.add(currentIndex);
            currentIndex = routeFrom[currentIndex];
        }
        int[] finalOutput = new int[output.size()];
        for(int i = 0; i < output.size(); i++) {
            finalOutput[i] = output.get(output.size() - i - 1);
        }
        return finalOutput;
    }
    private double[][] constructWeightsFromGraph(int[][] input) {
        int arraySize = input.length;
        int area = arraySize / 4;
        int dimension = (int)Math.round(Math.sqrt(area));
        double[][] output = new double[arraySize][];
        for(int i = 0; i < arraySize; i++) {
            int[] graphEdges = input[i];
            output[i] = new double[graphEdges.length];
            for(int j = 0; j < graphEdges.length; j++) {
                if(input[i][j] / 4 == i) {
                    output[i][j] = (Math.PI / 2) / rotationSpeedPerPower;
                } else {
                    output[i][j] = millimetersPerTableRow / linearSpeedPerPower;
                }
            }
        }
        return output;
    }
    private int[][] constructGraphFromTable(boolean[][] input) {
        int[][] trigTable = new int[][] {
                new int[] { 1, 0 },
                new int[] { 0, 1 },
                new int[] { -1, 0 },
                new int[] { 0, -1 }
        };
        int dimension = input[0].length;
        int elements = 4 * dimension * dimension;
        ArrayList<ArrayList<Integer>> tempTable = new ArrayList<>();
        for(int i = 0; i < elements; i++) {
            tempTable.add(new ArrayList<Integer>());
        }
        for(int i = 0; i < dimension; i++) {
            for(int j = 0; j < dimension; j++) {
                for(int k = 0; k < 4; k++) {
                    if(input[i][j]) continue;
                    int[][] outEdges = new int[][] {
                            new int[] { i, j, (k + 1) % 4 },
                            new int[] { i, j, Math.floorMod(k - 1, 4) },
                            new int[] { i + trigTable[k][0], j + trigTable[k][1], k }//,
                            //new int[] { i - trigTable[k][0], j - trigTable[k][1], k }
                    };
                    for(int[] outEdge : outEdges) {
                        if(outEdge[0] < 0 || outEdge[0] >= dimension || outEdge[1] < 0 || outEdge[1] > dimension) continue;
                        if(input[outEdge[0]][outEdge[1]]) continue;
                        int outNode = outEdge[2] + 4 * (outEdge[1] + dimension * outEdge[0]);
                        tempTable.get(k + 4 * (j + dimension * i)).add(outNode);
                    }
                }
            }
        }
        int[][] finalTable = new int[elements][];
        for(int i = 0; i < elements; i++) {
            int edgesSize = tempTable.get(i).size();
            finalTable[i] = new int[edgesSize];
            for(int j = 0; j < edgesSize; j++) {
                finalTable[i][j] = tempTable.get(i).get(j);
            }
        }
        return finalTable;
    }
    private int[] constructRouteFromTable(boolean[][] table, int startingRow, int startingColumn, int startingRotation, int endingRow, int endingColumn, int endingRotation) {
        int[][] edges = constructGraphFromTable(table);
        double[][] weights = constructWeightsFromGraph(edges);
        int dimension = table.length;
        int startingPoint = startingRotation + 4 * (startingColumn + dimension * startingRow);
        int endingPoint = endingRotation + 4 * (endingColumn + dimension * endingRow);
        return constructDjikstraRoute(edges.length, edges, weights, startingPoint, endingPoint);
    }
    private ArrayList<RobotMoveItem> constructMovesFromTable(boolean[][] table, int startingRow, int startingColumn, int startingRotation, int endingRow, int endingColumn, int endingRotation) {
        int[] output = constructRouteFromTable(table, startingRow, startingColumn, startingRotation, endingRow, endingColumn, endingRotation);
        int columns = table[0].length;
        int rows = table.length;
        int[][] trigTable = new int[][] {
                new int[] { 1, 0 },
                new int[] { 0, 1 },
                new int[] { -1, 0 },
                new int[] { 0, -1 }
        };
        ArrayList<RobotMoveItem> result = new ArrayList<>();
        for(int i = 1; i < output.length; i++) {
            int start = output[i - 1];
            int end = output[i];
            int startRotation = start % 4;
            int endRotation = end % 4;
            int startColumn = (start / 4) % columns;
            int endColumn = (end / 4) % columns;
            int startRow = start / (4 * columns);
            int endRow = end / (4 * columns);
            boolean isRotation = false;
            double magnitude = 0;
            if(startRotation == endRotation) {
                if(startColumn == endColumn) {
                    magnitude = 10.0 * (endRow - startRow) / trigTable[startRotation][0];
                } else {
                    magnitude = 10.0 * (endColumn - startColumn) / trigTable[startRotation][1];
                }
            } else {
                isRotation = true;
                if(endRotation == startRotation + 1 || (endRotation == 0 && startRotation == 3)) {
                    magnitude = Math.PI / 2;
                } else {
                    magnitude = -Math.PI / 2;
                }
            }
            result.add(new RobotMoveItem(isRotation, magnitude));
        }
        return result;
    }
    private void finalizeLoopStage() {
        telemetry.update();
        sleep(CYCLE_MS);
        idle();
    }
    private boolean updateLocations() {
        getCameraLocation();
        return getRobotLocation();
    }
    private void beginMainProgram() {
        getCameraLocation();
        waitForStart();
        targetsUltimateGoal.activate();

    }
    private void initializeMotors() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
    }
    private DcMotor[] getMotors(int frontBack, int rightLeft)
    {
        DcMotor[][][] returnArray = new DcMotor[][][] {
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLB
                        },
                        new DcMotor[] {
                                motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorLF, motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF
                        },
                        new DcMotor[] {
                                motorRF, motorLF
                        },
                        new DcMotor[] {
                                motorRF
                        }
                }
        };
        return returnArray[frontBack + 1][rightLeft + 1];
    }
    private void setMotors(int frontBack, int rightLeft, double amount) {
        boolean isPositive = amount >= 0;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        if(!isPositive) direction = DcMotorSimple.Direction.REVERSE;
        double correctedAmount = Math.abs(amount);
        DcMotor[] motors = getMotors(frontBack, rightLeft);
        for(DcMotor motor : motors) {
            motor.setDirection(direction);
            motor.setPower(correctedAmount);
        }
    }
    private void beginLinearMotion(double power) {
        setMotors(0, 0, power);
    }
    private void beginRotation(double power) {
        setMotors(0, 1, power);
        setMotors(0, -1, -power);
    }
    private void stopRobot()
    {
        stopTime = LocalDateTime.now();
        setMotors(0, 0, 0);
    }
    private void moveLinearInternal(double power, double millimeters) {
        beginLinearMotion(power);
        setStopTime(power, millimeters, linearSpeedPerPower);
    }
    private void rotateInternal(double power, double radians) {
        beginRotation(power);
        setStopTime(power, radians, rotationSpeedPerPower);
    }
    private void moveLinear(double millimeters) {
        if(!stopIfZero(millimeters)) moveLinearInternal(getPowerFromDistance(millimeters), millimeters);
    }
    private void rotate(double radians) {
        if(!stopIfZero(radians)) rotateInternal(getPowerFromDistance(radians), radians);
    }
    private boolean stopIfZero(double distance) {
        if(distance == 0.0) {
            stopRobot();
            return true;
        }
        return false;
    }
    private double getPowerFromDistance(double distance) {
        double power = 1;
        if(distance < 0) power = -1;
        return power;
    }
    private void setStopTime(double power, double distance, double speedPerPower) {
        double seconds = Math.abs(distance / (speedPerPower * power));
        stopTime = LocalDateTime.now().plusNanos((long)(seconds * 1000000000.0));
    }
    private void getCameraLocation() {
        float phoneYRotate = 0;
        float phoneXRotate = 0;
        float phoneZRotate = 0;
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, cameraParameters.cameraDirection);
        }
    }
    private boolean getRobotLocation() {
        ArrayList<OpenGLMatrix> allVisible = new ArrayList<OpenGLMatrix>();
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
                allVisible.add(robotLocationTransform);
            }
        }
        if (!allVisible.isEmpty()) {
            translation = new VectorF(0, 0, 0);
            for (OpenGLMatrix visible : allVisible) {
                translation.add(visible.getTranslation());
            }
            translation.multiply(1f / allVisible.size());
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            VectorF rotationTotal = new VectorF(0, 0, 0);
            for (OpenGLMatrix visible : allVisible) {
                Orientation singleRotation = Orientation.getOrientation(visible, EXTRINSIC, XYZ, DEGREES);
                rotationTotal.add(new VectorF(singleRotation.firstAngle, singleRotation.secondAngle, singleRotation.thirdAngle));
            }
            rotation = new Orientation(EXTRINSIC, XYZ, DEGREES,
                    rotationTotal.get(0) / allVisible.size(),
                    rotationTotal.get(1) / allVisible.size(),
                    rotationTotal.get(2) / allVisible.size(), System.nanoTime());
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            return true;
        }
        telemetry.addData("Visible Target", "none");
        return false;
    }
    private void actOnMoveItem(RobotMoveItem item) {
        if(item.isRotation) {
            rotate(item.magnitude);
        } else {
            moveLinear(item.magnitude);
        }
    }
    private void shutdownProgram() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
        targetsUltimateGoal.deactivate();
    }
    private void transform(VuforiaTrackable trackable, float locationX, float locationY, float locationZ, float angle1, float angle2, float angle3)
    {
        trackable.setLocation(OpenGLMatrix
                .translation(locationX, locationY, locationZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, angle1, angle2, angle3)));
    }
    private void transform(VuforiaTrackable trackable, float[] values)
    {
        transform(trackable, values[0], values[1], values[2], values[3], values[4], values[5]);
    }
    private class DjikstraPriorityQueueItem {
        public int index;
        public double distance;
        public DjikstraPriorityQueueItem(int index, double distance) {
            this.index = index;
            this.distance = distance;
        }
    }
    private class DjikstraPriorityQueueComparator implements Comparator<DjikstraPriorityQueueItem> {

        @Override
        public int compare(DjikstraPriorityQueueItem item1, DjikstraPriorityQueueItem item2) {
            return Double.compare(item1.distance, item2.distance);
        }
    }
    private class RobotMoveItem {
        public boolean isRotation;
        public double magnitude;
        public RobotMoveItem(boolean isRotation, double magnitude) {
            this.isRotation = isRotation;
            this.magnitude = magnitude;
        }
    }
}
