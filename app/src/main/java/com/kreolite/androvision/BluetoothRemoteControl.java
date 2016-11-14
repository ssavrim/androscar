package com.kreolite.androvision;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.Toast;

import app.akexorcist.bluetotohspp.library.BluetoothSPP;
import app.akexorcist.bluetotohspp.library.BluetoothSPP.BluetoothConnectionListener;
import app.akexorcist.bluetotohspp.library.BluetoothState;
import app.akexorcist.bluetotohspp.library.DeviceList;

public class BluetoothRemoteControl extends AppCompatActivity {
    public static final String TAG = "RemoteControl";

    BluetoothSPP bt;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bluetooth_remote_control);
        bt = new BluetoothSPP(this);
        if(!bt.isBluetoothAvailable()) {
            Toast.makeText(getApplicationContext()
                    , "Bluetooth is not available"
                    , Toast.LENGTH_SHORT).show();
            finish();
        }

        bt.setBluetoothConnectionListener(new BluetoothConnectionListener() {
            public void onDeviceConnected(String name, String address) {
                Toast.makeText(getApplicationContext()
                        , "Connected to " + name
                        , Toast.LENGTH_SHORT).show();
            }

            public void onDeviceDisconnected() {
                Toast.makeText(getApplicationContext()
                        , "Connection lost"
                        , Toast.LENGTH_SHORT).show();
            }

            public void onDeviceConnectionFailed() {
                Log.i("Check", "Unable to connect");
            }
        });

        bt.setAutoConnectionListener(new BluetoothSPP.AutoConnectionListener() {
            public void onNewConnection(String name, String address) {
                Log.i("Check", "New Connection - " + name + " - " + address);
            }

            public void onAutoConnectionStarted() {
                Log.i("Check", "Auto menu_connection started");
            }
        });

        Button btnConnect = (Button)findViewById(R.id.btnConnect);
        btnConnect.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                if(bt.getServiceState() == BluetoothState.STATE_CONNECTED) {
                    bt.disconnect();
                } else {
                    Intent intent = new Intent(getApplicationContext(), DeviceList.class);
                    startActivityForResult(intent, BluetoothState.REQUEST_CONNECT_DEVICE);
                }
            }
        });
    }
    public void onDestroy() {
        super.onDestroy();
        bt.stopService();
    }

    public void onStart() {
        super.onStart();
        if(!bt.isBluetoothEnabled()) {
            bt.enable();
        } else {
            if(!bt.isServiceAvailable()) {
                bt.setupService();
                bt.startService(BluetoothState.DEVICE_ANDROID);
                setup();
            }
        }
    }

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(requestCode == BluetoothState.REQUEST_CONNECT_DEVICE) {
            if(resultCode == AppCompatActivity.RESULT_OK)
                bt.connect(data);
        } else if(requestCode == BluetoothState.REQUEST_ENABLE_BT) {
            if(resultCode == AppCompatActivity.RESULT_OK) {
                bt.setupService();
            } else {
                Toast.makeText(getApplicationContext()
                        , "Bluetooth was not enabled."
                        , Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    public void setup() {
        Button btnUp = (Button)findViewById(R.id.btnUp);
        btnUp.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                bt.send("8", true);
            }
        });
        Button btnDown = (Button)findViewById(R.id.btnDown);
        btnDown.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                bt.send("2", true);
            }
        });
        Button btnLeft = (Button)findViewById(R.id.btnLeft);
        btnLeft.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                bt.send("4", true);
            }
        });
        Button btnRight = (Button)findViewById(R.id.btnRight);
        btnRight.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                bt.send("6", true);
            }
        });
        Button btnStop = (Button)findViewById(R.id.btnStop);
        btnStop.setOnClickListener(new OnClickListener(){
            public void onClick(View v){
                bt.send("0", true);
            }
        });
        bt.autoConnect("IOIO");
    }
}
