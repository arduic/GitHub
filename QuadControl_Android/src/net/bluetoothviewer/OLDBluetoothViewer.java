/* /*
 * Copyright (C) 2010 Janos Gyerik
 *
 * This file is part of BluetoothViewer.
 *
 * BluetoothViewer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BluetoothViewer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BluetoothViewer.  If not, see <http://www.gnu.org/licenses/>.
 * /

package net.bluetoothviewer;

//import java.nio.ByteBuffer;
//import java.util.Timer;
//import java.util.TimerTask;

import net.bluetoothviewer.R;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * This is the main Activity that displays the current session.
 * /
public class BluetoothViewer extends Activity implements SensorEventListener {
    // Debugging
    private static final String TAG = "BluetoothViewer";
    private static final boolean D = true;

    // Message types sent from the BluetoothService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;

    // Key names received from the BluetoothService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";

    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;

    // Layout Views
    private TextView mTitle;
    private ListView mConversationView; 
    private EditText mOutEditText;
    private Button mSendButton;
    private View mSendTextContainer;
    private SeekBar mThrustBar;
    DrawView mdrawView;
    
    public int xdraw;
    public int ydraw;
    
    // Toolbar
    private ImageButton mToolbarConnectButton;
    private ImageButton mToolbarDisconnectButton;
    private ImageButton mToolbarPauseButton;
    private ImageButton mToolbarPlayButton;

    // Name of the connected device
    private String mConnectedDeviceName = null;
    // Array adapter for the conversation thread
    private ArrayAdapter<String> mConversationArrayAdapter;
    // String buffer for outgoing messages
    private StringBuffer mOutStringBuffer;
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    // Member object for the Bluetooth services
    private BluetoothChatService mBluetoothService = null;
    
    //MISC
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;

    // State variables
    private boolean paused = false;
    private boolean connected = false;
    

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
          
        if(D) Log.e(TAG, "+++ ON CREATE +++");
              
        // Set up the window layout
        requestWindowFeature(Window.FEATURE_CUSTOM_TITLE);
        setContentView(R.layout.main);
        getWindow().setFeatureInt(Window.FEATURE_CUSTOM_TITLE, R.layout.custom_title);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        // Set up the custom title
        mTitle = (TextView) findViewById(R.id.title_left_text);
        mTitle.setText(R.string.app_name);
        mTitle = (TextView) findViewById(R.id.title_right_text);
        
        mSendTextContainer = (View) findViewById(R.id.send_text_container);
        
        mToolbarConnectButton = (ImageButton) findViewById(R.id.toolbar_btn_connect);
        mToolbarConnectButton.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		startDeviceListActivity();
        	}
        });

        mToolbarDisconnectButton = (ImageButton) findViewById(R.id.toolbar_btn_disconnect);
        mToolbarDisconnectButton.setOnClickListener(new OnClickListener(){
			public void onClick(View v) {
				disconnectDevices();
			}
        });
        
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
                
        mToolbarPauseButton = (ImageButton) findViewById(R.id.toolbar_btn_pause);
        mToolbarPauseButton.setOnClickListener(new OnClickListener(){
			public void onClick(View v) {
				paused = true;
				onPausedStateChanged();
			}
        });
        
        mToolbarPlayButton = (ImageButton) findViewById(R.id.toolbar_btn_play);
        mToolbarPlayButton.setOnClickListener(new OnClickListener(){
			public void onClick(View v) {
				paused = false;
				onPausedStateChanged();
			}
        });
        
        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        
    }
    
    private void startDeviceListActivity() {
        Intent serverIntent = new Intent(this, DeviceListActivity.class);
        startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
    }

    @Override
    public void onStart() {
        super.onStart();
        if(D) Log.e(TAG, "++ ON START ++");

        // If BT is not on, request that it be enabled.
        // setupUserInterface() will then be called during onActivityResult
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
        // Otherwise, setup the Bluetooth session
        } else {
            if (mBluetoothService == null) setupUserInterface();
        }
    }

    @Override
    public synchronized void onResume() {
        super.onResume();
        if(D) Log.e(TAG, "+ ON RESUME +");
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_GAME);
    }
    
    private void setupUserInterface() { 
        Log.d(TAG, "setupUserInterface()");

        // Initialize the array adapter for the conversation thread
        mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message);
        mConversationView = (ListView) findViewById(R.id.in);
        mConversationView.setAdapter(mConversationArrayAdapter);
        
        mConversationArrayAdapter.add("Welcome to QuadControl!");
        mConversationArrayAdapter.add("This application gives you a new way to control your quadcopter! ");
        mConversationArrayAdapter.add("Use the toolbar below to manage connections and get started.");
        mConversationArrayAdapter.add("You should receive a pairing request the first time the phone and command module are connected.  Make sure to accept this request so that the devices can communicate.");
        mConversationArrayAdapter.add("If the device ever fails to connect, reboot the command module and try again.");
        

        // Initialize the compose field with a listener for the return key
        mOutEditText = (EditText) findViewById(R.id.edit_text_out);
        mOutEditText.setOnEditorActionListener(mWriteListener);

        // Initialize the send button with a listener that for click events
        mSendButton = (Button) findViewById(R.id.button_send);
        mSendButton.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                // Send a message using content of the edit text widget
                TextView view = (TextView) findViewById(R.id.edit_text_out);
                String message = view.getText().toString();
//                sendMessage(message);
            }
        });
        
        // Initialize the thrust bar with a listener for change events
//        mThrustBar = (SeekBar) findViewById(R.id.ThrustBar);
//        mThrustBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
//            public void onProgressChanged(SeekBar SeekBar, int progress, boolean fromUser) {
//                sendMessage();
//            }

//			@Override
//			public void onStartTrackingTouch(SeekBar seekBar) {
				
//			}

//			@Override
//			public void onStopTrackingTouch(SeekBar seekBar) {
//				mThrustBar.setProgress(50);
//				sendMessage();
//			}
//        });
        
        // Initialize the BluetoothService to perform Bluetooth connections
        mBluetoothService = new BluetoothChatService(mHandler);

        // Initialize the buffer for outgoing messages
        mOutStringBuffer = new StringBuffer("");
        
        onBluetoothStateChanged();
    }

    @Override
    public synchronized void onPause() {
        super.onPause();
        if(D) Log.e(TAG, "- ON PAUSE -");
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onStop() {
        super.onStop();
        if(D) Log.e(TAG, "-- ON STOP --");
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // Stop the Bluetooth services
        if (mBluetoothService != null) mBluetoothService.stop();
        if(D) Log.e(TAG, "--- ON DESTROY ---");
    }
    
    /**
     * Sends a message.
     * @param message  A string of text to send.
     * /
    private void sendMessage() {
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothChatService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

/*        // Check that there's actually something to send
        if (message.length() > 0) {
        	message += "\n";
            // Get the message bytes and tell the BluetoothService to write
            byte[] send = message.getBytes();
            //mBluetoothService.write(send);
			
            // Reset out string buffer to zero and clear the edit text field
            mOutStringBuffer.setLength(0);
            mOutEditText.setText(mOutStringBuffer);
        }
* /      
       	double power=207-2.05*mThrustBar.getProgress();
    	int throttle=(int)power;
        mBluetoothService.write(intToByteArray(0xFA,0xEB,throttle,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11));
    }
        
    
    public static byte[] intToByteArray(int a ,int b, int c, int d, int e, int f, int g, int h, int i, int j, int k)
    {
        byte[] ret = new byte[11];
        ret[0] = (byte) (a & 0xFF); 
        ret[1] = (byte) (b & 0xFF);
        ret[2] = (byte) (c & 0xFF);
        ret[3] = (byte) (d & 0xFF);
        ret[4] = (byte) (e & 0xFF);
        ret[5] = (byte) (f & 0xFF);
        ret[6] = (byte) (g & 0xFF);
        ret[7] = (byte) (h & 0xFF);
        ret[8] = (byte) (i & 0xFF);
        ret[9] = (byte) (j & 0xFF);
        ret[10] = (byte) (k & 0xFF);
        
        return ret;
    }
    
    String ByteArraytoString(byte[] bytes) {
    	StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return(sb.toString());
   }

    // The action listener for the EditText widget, to listen for the return key
    private TextView.OnEditorActionListener mWriteListener =
        new TextView.OnEditorActionListener() {
        public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
            // If the action is a key-up event on the return key, send the message
            if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
                String message = view.getText().toString();
                sendMessage();
            }
            if(D) Log.i(TAG, "END onEditorAction");
            return true;
        }
    };

    // The Handler that gets information back from the BluetoothService
    private final Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
            case MESSAGE_STATE_CHANGE:
                if(D) Log.i(TAG, "MESSAGE_STATE_CHANGE: " + msg.arg1);
                switch (msg.arg1) {
                case BluetoothChatService.STATE_CONNECTED:
                	connected = true;
                    mTitle.setText(mConnectedDeviceName);
                    break;
                case BluetoothChatService.STATE_CONNECTING:
                    mTitle.setText(R.string.title_connecting);
                    break;
                case BluetoothChatService.STATE_NONE:
                	connected = false;
                    mTitle.setText(R.string.title_not_connected);
                    break;
                }
                onBluetoothStateChanged();
                break;
            case MESSAGE_WRITE:
                byte[] writeBuf = (byte[]) msg.obj;
                // construct a string from the buffer
//                String writeMessage = new String(writeBuf);
//                mConversationArrayAdapter.add(">>> " + writeMessage);
                
//                mConversationArrayAdapter.add(ByteArraytoString(writeBuf));
                if (D) Log.d(TAG, "written = '" + ByteArraytoString(writeBuf) + "'");
                break;
            case MESSAGE_READ:
//            	if (paused) break;
                byte[] readBuf = (byte[]) msg.obj;
                // construct a string from the valid bytes in the buffer
                String readMessage = new String(readBuf, 0, msg.arg1);
                if (D) Log.d(TAG, readMessage);
//                mConversationArrayAdapter.add(readMessage);
                break;
            case MESSAGE_DEVICE_NAME:
                // save the connected device's name
                mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
                Toast.makeText(getApplicationContext(), "Connected to "
                               + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
                break;
            case MESSAGE_TOAST:
                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
                               Toast.LENGTH_SHORT).show();
                break;
            }
        }
    };

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(D) Log.d(TAG, "onActivityResult " + resultCode);
        switch (requestCode) {
        case REQUEST_CONNECT_DEVICE:
            // When DeviceListActivity returns with a device to connect
            if (resultCode == Activity.RESULT_OK) {
                // Get the device MAC address
                String address = data.getExtras()
                                     .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
                // Get the BLuetoothDevice object
                BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                // Attempt to connect to the device
                mBluetoothService.connect(device);
            }
            break;
        case REQUEST_ENABLE_BT:
            // When the request to enable Bluetooth returns
            if (resultCode == Activity.RESULT_OK) {
                // Bluetooth is now enabled, so set up a Bluetooth session
                setupUserInterface();
            } else {
                // User did not enable Bluetooth or an error occurred
                Log.d(TAG, "BT not enabled");
                Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        
        return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.menu_quit:
        	this.finish();
        }
        return false;
    }

    private void disconnectDevices() {
    	if (mBluetoothService != null) mBluetoothService.stop();
    	
    	onBluetoothStateChanged();
    }
    
    private void onBluetoothStateChanged() {
    	if (connected) {
			mToolbarConnectButton.setVisibility(View.GONE);
			mToolbarDisconnectButton.setVisibility(View.VISIBLE);
//			mThrustBar.setVisibility(View.VISIBLE);
			mSendTextContainer.setVisibility(View.VISIBLE);
    	}
    	else {
			mToolbarConnectButton.setVisibility(View.VISIBLE);
			mToolbarDisconnectButton.setVisibility(View.GONE);
//			mThrustBar.setVisibility(View.VISIBLE);
			mSendTextContainer.setVisibility(View.GONE); 
    	}
		paused = false;
    	onPausedStateChanged();
    }

    private void onPausedStateChanged() {
    	if (connected) {
	    	if (paused) {
	    		mToolbarPlayButton.setVisibility(View.VISIBLE);
	    		mToolbarPauseButton.setVisibility(View.GONE);
	    	}
	    	else {
	    		mToolbarPlayButton.setVisibility(View.GONE);
	    		mToolbarPauseButton.setVisibility(View.VISIBLE);
	    	}
    	}
    	else {
    		mToolbarPlayButton.setVisibility(View.GONE);
    		mToolbarPauseButton.setVisibility(View.GONE);
    	}
    }

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		double dox = 9*event.values[1]+90;
		int inx = (int)dox;
		double doy = 9*event.values[0]+90;
		int iny = (int)doy;
		double power=207-2.05*mThrustBar.getProgress();
    	int throttle=(int)power;
    	Log.d(TAG,""+inx+"     "+iny);
    	xdraw=360+inx;
    	ydraw=360+iny;
        mBluetoothService.write(intToByteArray(0xFA,0xEB,throttle,inx,iny,0x11,0x11,0x11,0x11,0x11,0x11));
        
//		  mdrawView = new DrawView(this,xdraw,ydraw);
        
//        mdrawView.setBackgroundColor(Color.TRANSPARENT);
//        setContentView(mdrawView);
	}    
}
*/



