package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class RecordingPipeline extends OpenCvPipeline {
    OpenCvWebcam camera;
    public RecordingPipeline(OpenCvWebcam camera) {
        this.camera = camera;
    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    @Override
    public void init(Mat input) {
        camera.startRecordingPipeline(
                new PipelineRecordingParameters.Builder()
                        .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                        .setEncoder(PipelineRecordingParameters.Encoder.H264)
                        .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                        .setFrameRate(30)
                        .setPath(Environment.getExternalStorageDirectory() + "/pipeline_rec_" + ".mp4")
                        .build());
    }
}

