package org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

// field 142 x 142 inches (6 tiles x 6 tiles)
// tile 24 x 24 inches
// 3rd tile from pixel drop area are bars
// 8 inches from wall (backboard)
// 12 inches from backboard to blue tape
// 0.5 inch for half of metal bar

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    // roadrunner things
    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    // camera stuff
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        // init tensor flow
        initTfod();

        // init sample mecanum drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // run the roadrunner path created
        RRrunPath(drive);

        // wait for user input to start
        waitForStart();

        // robot movement below
        //moveForward(0.75, 2000); // forward for 2 seconds
        //strafeLeft(0.75, 2000); // strafe left for 2 seconds

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    /*
     * autonomous movement using Roadrunner
     * NOTE: ALL DISTANCES IN THESE FUNCTIONS ARE IN INCHES
     */

    // roadrunner path
    public void RRrunPath(SampleMecanumDrive drive) {
        // spline motion from (0, 0) to (10, 10)
        Trajectory myTraj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 0), 0)
                .splineTo(new Vector2d(10, 10), 0)
                .build();

        drive.followTrajectory(myTraj);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}
