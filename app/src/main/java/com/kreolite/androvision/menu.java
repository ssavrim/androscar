package com.kreolite.androvision;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;

public class menu extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_menu);
    }

    public void openCamera(View view) {
        Intent intent = new Intent(this, camera.class);
        startActivity(intent);
    }
    public void blobDetect(View view) {
        Intent intent = new Intent(this, blob.class);
        startActivity(intent);
    }
    public void faceDetect(View view) {
        Intent intent = new Intent(this, facedetect.class);
        startActivity(intent);
    }
    public void trackMotion(View view) {
        Intent intent = new Intent(this, motiontracker.class);
        startActivity(intent);
    }
    public void carTracker(View view) {
        Intent intent = new Intent(this, carTracker.class);
        startActivity(intent);
    }
}
