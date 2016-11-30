package com.kreolite.androvision;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.net.nsd.NsdServiceInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.design.widget.Snackbar;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.text.format.Formatter;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.io.IOException;
import java.io.RandomAccessFile;

public class MainActivity extends AppCompatActivity implements ActivityCompat.OnRequestPermissionsResultCallback {
    public static final String TAG = "MainActivity";

    /**
     * Id to identify a camera permission request.
     */
    public NsdServiceInfo mChoosenService = null;
    private static final int REQUEST_CAMERA = 0;
    private NsdHelper mNsdHelper;
    private BroadcastReceiver mBroadcastReceiver;
    /**
     * Root of the layout of this Activity.
     */
    private View mLayout;
    private LinearLayout mDynamicLayout;
    private TextView mIpAddressView;
    private String mDeviceIpAddress;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mLayout = findViewById(R.id.main_layout);
        mDynamicLayout = (LinearLayout) findViewById(R.id.dynamic_layout);
        mIpAddressView = (TextView) findViewById(R.id.device_ip_address);
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            // Camera permission has not been granted.
            requestCameraPermission();
        }
        WifiManager wifiManager = (WifiManager) getSystemService(WIFI_SERVICE);
        mDeviceIpAddress = Formatter.formatIpAddress(wifiManager.getConnectionInfo().getIpAddress());
        mIpAddressView.setText(mDeviceIpAddress);
        try {
            String content = (new RandomAccessFile("/data/androvision_default_mode.txt", "r")).readLine();
            Log.d("menu", String.format("onCreate: %s",content));
            switch (content){
                case "remoteControl":
                    remoteControl(null);
                    break;
                case "carView":
                    carView(null);
                    break;
                case "carViewGPIO":
                    carViewGPIO(null);
                    break;
            }
        } catch (IOException e) {
                // ignore
        }
        mNsdHelper = new NsdHelper(this);
        mNsdHelper.initializeNsd();
        mBroadcastReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                mDynamicLayout.removeAllViews();
                for(final NsdServiceInfo info:mNsdHelper.getServiceInfos()){
                    Button b = new Button(MainActivity.this);
                    b.setText("remote control " + info);
                    b.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            mChoosenService = info;
                            remoteControl(view);
                        }
                    });
                    mDynamicLayout.addView(b);

                }
            }
        };

    }
    /**
     * Requests the Camera permission.
     * If the permission has been denied previously, a SnackBar will prompt the user to grant the
     * permission, otherwise it is requested directly.
     */
    private void requestCameraPermission() {
        Log.i(TAG, "CAMERA permission has NOT been granted. Requesting permission.");

        if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.CAMERA)) {
            // Provide an additional rationale to the user if the permission was not granted
            // and the user would benefit from additional context for the use of the permission.
            // For example if the user has previously denied the permission.
            Log.i(TAG,
                    "Displaying camera permission rationale to provide additional context.");
            Snackbar.make(mLayout, R.string.permission_camera_rationale,
                    Snackbar.LENGTH_INDEFINITE)
                    .setAction(R.string.allow, new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            ActivityCompat.requestPermissions(MainActivity.this,
                                    new String[]{Manifest.permission.CAMERA},
                                    REQUEST_CAMERA);
                        }
                    })
                    .show();
        } else {

            // Camera permission has not been granted yet. Request it directly.
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA);
        }
    }
    @Override
    protected void onPause() {

        if (mNsdHelper != null) {
            mNsdHelper.stopDiscovery();
        }
        if (mBroadcastReceiver != null) {
            unregisterReceiver(mBroadcastReceiver);
        }
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mNsdHelper != null) {
            mNsdHelper.discoverServices();
        }
        if (mBroadcastReceiver != null){
            registerReceiver(mBroadcastReceiver, new IntentFilter(mNsdHelper.FOUND_INTENT));
        }
    }

    public void remoteControl(View view) {
        Intent intent = new Intent(this, RosRemoteControlActivity.class);
        if (mChoosenService != null) {
            Bundle b = new Bundle();
            b.putString(RosRemoteControlActivity.MASTER_URI_EXTRA,
                    "http:/" + mChoosenService.getHost() + ":" + mChoosenService.getPort() + "/");

            intent.putExtra(RosRemoteControlActivity.MASTER_URI_EXTRA, b);
        }
        startActivity(intent);
    }
    public void carView(View view) {
        Intent intent = new Intent(this, RosCameraActivity.class);
        startActivity(intent);
    }
    public void carViewGPIO(View view) {
        Intent intent = new Intent(this, RosCameraGPIOActivity.class);
        startActivity(intent);
    }
    /**
     * Callback received when a permissions request has been completed.
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {

        if (requestCode == REQUEST_CAMERA) {
            // BEGIN_INCLUDE(permission_result)
            // Received permission result for camera permission.
            Log.i(TAG, "Received response for Camera permission request.");

            // Check if the only required permission has been granted
            if (grantResults.length == 1 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Camera permission has been granted, preview can be displayed
                Log.i(TAG, "CAMERA permission has now been granted. Showing preview.");
                Snackbar.make(mLayout, R.string.permision_available_camera,
                        Snackbar.LENGTH_SHORT).show();
            } else {
                Log.i(TAG, "CAMERA permission was NOT granted.");
                Snackbar.make(mLayout, R.string.permissions_not_granted,
                        Snackbar.LENGTH_SHORT).show();
            }
            // END_INCLUDE(permission_result)

        } else {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }

}
