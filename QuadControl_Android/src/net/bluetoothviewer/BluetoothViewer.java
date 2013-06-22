/*
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
 */

package net.bluetoothviewer;

import net.bluetoothviewer.R;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
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
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.widget.Toast;

/**
 * This is the main Activity that displays the current session.
 */
public class BluetoothViewer extends Activity implements SensorEventListener{
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
//    private ListView mConversationView;
//    private EditText mOutEditText;
//    private Button mSendButton;
//    private View mSendTextContainer;
    private SeekBar mThrustBar;
    private SeekBar mYawBar;
    private TextView mThrustTitle;
    private TextView mYawTitle;
    private Button mAssist;
    private Button mManual;
    private Button mLaunch;
    private Button mControl;
    private Button mLand;
    private Button mGround;    
    private TextView mA_Label;
    private TextView mA_Text;
    private Button mA_Plus;
    private Button mA_Minus;
    private TextView mB_Label;
    private TextView mB_Text;
    private Button mB_Plus;
    private Button mB_Minus;
    private TextView mC_Label;
    private TextView mC_Text;
    private Button mC_Plus;
    private Button mC_Minus;
    private TextView mD_Label;
    private TextView mD_Text;
    private Button mD_Plus;
    private Button mD_Minus;
    private TextView mp_Label;
    private TextView mp_Text;
    private Button mp_Plus;
    private Button mp_Minus;
    private TextView mi_Label;
    private TextView mi_Text;
    private Button mi_Plus;
    private Button mi_Minus;
    private TextView md_Label;
    private TextView md_Text;
    private Button md_Plus;
    private Button md_Minus;
    

    public int flightMode = FLIGHT_MODE_GROUND;
    
    public static final int FLIGHT_MODE_GROUND=0;
    public static final int FLIGHT_MODE_ASSIST=1;
    public static final int FLIGHT_MODE_MANUAL=2;
    public static final int FLIGHT_MODE_LAUNCH=3;
    public static final int FLIGHT_MODE_LAND=4;
    
    //Toolbar
    private ImageButton mToolbarConnectButton;
    private ImageButton mToolbarDisconnectButton;
//    private ImageButton mToolbarPauseButton;
//    private ImageButton mToolbarPlayButton;
    
    //MISC
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;

    // Name of the connected device
    private String mConnectedDeviceName = null;
    // Array adapter for the conversation thread
//    private ArrayAdapter<String> mConversationArrayAdapter;
    // String buffer for outgoing messages
//    private StringBuffer mOutStringBuffer;
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    // Member object for the Bluetooth services
    private BluetoothChatService mBluetoothService = null;

    // State variables
    private boolean paused = false;
    private boolean connected = false;
    public int xdraw;
    public int ydraw;
    public int offsetA=0;
    public int offsetB=0;
    public int offsetC=0;
    public int offsetD=0;
    public int motorA=0;
    public int motorB=0;
    public int motorC=0;
    public int motorD=0;
    public int p=0;
    public int i=0;
    public int d=0;

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
        
        mThrustTitle=(TextView) findViewById(R.id.ThrustLabel);
        mYawTitle=(TextView) findViewById(R.id.YawLabel);
        mA_Label=(TextView) findViewById(R.id.A_Label);
        mA_Text=(TextView) findViewById(R.id.A_Text);
        mB_Label=(TextView) findViewById(R.id.B_Label);
        mB_Text=(TextView) findViewById(R.id.B_Text);
        mC_Label=(TextView) findViewById(R.id.C_Label);
        mC_Text=(TextView) findViewById(R.id.C_Text);
        mD_Label=(TextView) findViewById(R.id.D_Label);
        mD_Text=(TextView) findViewById(R.id.D_Text);
        mp_Label=(TextView) findViewById(R.id.p_Label);
        mp_Text=(TextView) findViewById(R.id.p_Text);
        mi_Label=(TextView) findViewById(R.id.i_Label);
        mi_Text=(TextView) findViewById(R.id.i_Text);
        md_Label=(TextView) findViewById(R.id.d_Label);
        md_Text=(TextView) findViewById(R.id.d_Text);
        
        mA_Plus=(Button) findViewById(R.id.A_Plus);
        mA_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetA++;
        	}
        });
        
        mA_Minus=(Button) findViewById(R.id.A_Minus);
        mA_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetA--;
        	}
        });
        
        mB_Plus=(Button) findViewById(R.id.B_Plus);
        mB_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetB++;
        	}
        });
        
        mB_Minus=(Button) findViewById(R.id.B_Minus);
        mB_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetB--;
        	}
        });
        
        mC_Plus=(Button) findViewById(R.id.C_Plus);
        mC_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetC++;
        	}
        });
        
        mC_Minus=(Button) findViewById(R.id.C_Minus);
        mC_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetC--;
        	}
        });
        
        mD_Plus=(Button) findViewById(R.id.D_Plus);
        mD_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetD++;
        	}
        });
        
        mD_Minus=(Button) findViewById(R.id.D_Minus);
        mD_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		offsetD--;
        	}
        });
        
        mp_Plus=(Button) findViewById(R.id.p_Plus);
        mp_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		p++;
        	}
        });
        
        mp_Minus=(Button) findViewById(R.id.p_Minus);
        mp_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		p--;
        	}
        });
        
        mi_Plus=(Button) findViewById(R.id.i_Plus);
        mi_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		i++;
        	}
        });
        
        mi_Minus=(Button) findViewById(R.id.i_Minus);
        mi_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		i--;
        	}
        });
        
        md_Plus=(Button) findViewById(R.id.d_Plus);
        md_Plus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		d++;
        	}
        });
        
        md_Minus=(Button) findViewById(R.id.d_Minus);
        md_Minus.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		d--;
        	}
        });
        
        mAssist=(Button) findViewById(R.id.btn_Assist);
        mAssist.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		mToolbarDisconnectButton.setVisibility(View.GONE);
        		mAssist.setVisibility(View.GONE);
        		mManual.setVisibility(View.GONE);
        		mLaunch.setVisibility(View.VISIBLE);
        		mControl.setVisibility(View.GONE);
        		mLand.setVisibility(View.GONE);
        		mGround.setVisibility(View.VISIBLE);
        		mThrustBar.setVisibility(View.GONE);
    			mYawBar.setVisibility(View.GONE);
    			mThrustTitle.setVisibility(View.GONE);
    			mYawTitle.setVisibility(View.GONE);
    			flightMode=FLIGHT_MODE_GROUND;  		
        	}
        });
        
        mManual=(Button) findViewById(R.id.btn_Manual);
        mManual.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		mToolbarDisconnectButton.setVisibility(View.GONE);
        		mAssist.setVisibility(View.GONE);
        		mManual.setVisibility(View.GONE);
        		mLaunch.setVisibility(View.GONE);
        		mControl.setVisibility(View.GONE);
        		mLand.setVisibility(View.GONE);
        		mGround.setVisibility(View.VISIBLE);
        		mThrustBar.setVisibility(View.VISIBLE);
    			mYawBar.setVisibility(View.VISIBLE);
    			mThrustTitle.setVisibility(View.VISIBLE);
    			mYawTitle.setVisibility(View.VISIBLE);
    			flightMode=FLIGHT_MODE_MANUAL;
    			mThrustBar.setProgress(0);
        	}
        });
        
        mLaunch=(Button) findViewById(R.id.btn_Launch);
        mLaunch.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		mToolbarDisconnectButton.setVisibility(View.GONE);
        		mAssist.setVisibility(View.GONE);
        		mManual.setVisibility(View.GONE);
        		mLaunch.setVisibility(View.GONE);
        		mControl.setVisibility(View.VISIBLE);
        		mLand.setVisibility(View.GONE);
        		mGround.setVisibility(View.GONE);
        		mThrustBar.setVisibility(View.GONE);
    			mYawBar.setVisibility(View.GONE);
    			mThrustTitle.setVisibility(View.GONE);
    			mYawTitle.setVisibility(View.GONE);
    			flightMode=FLIGHT_MODE_LAUNCH;  
        	}
        });
        
        mControl=(Button) findViewById(R.id.btn_Control);
        mControl.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		mToolbarDisconnectButton.setVisibility(View.GONE);
        		mAssist.setVisibility(View.GONE);
        		mManual.setVisibility(View.GONE);
        		mLaunch.setVisibility(View.GONE);
        		mControl.setVisibility(View.GONE);
        		mLand.setVisibility(View.VISIBLE);
        		mGround.setVisibility(View.GONE);
        		mThrustBar.setVisibility(View.VISIBLE);
    			mYawBar.setVisibility(View.VISIBLE);
    			mThrustTitle.setVisibility(View.VISIBLE);
    			mYawTitle.setVisibility(View.VISIBLE);
    			flightMode=FLIGHT_MODE_ASSIST;  
    			mThrustBar.setProgress(50);
        	}
        });
        
        mLand=(Button) findViewById(R.id.btn_Land);
        mLand.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
        		mToolbarDisconnectButton.setVisibility(View.GONE);
        		mAssist.setVisibility(View.GONE);
        		mManual.setVisibility(View.GONE);
        		mLaunch.setVisibility(View.GONE);
        		mControl.setVisibility(View.VISIBLE);
        		mLand.setVisibility(View.GONE);
        		mGround.setVisibility(View.VISIBLE);
        		mThrustBar.setVisibility(View.VISIBLE);
    			mYawBar.setVisibility(View.GONE);
    			mThrustTitle.setVisibility(View.VISIBLE);
    			mYawTitle.setVisibility(View.GONE);
    			flightMode=FLIGHT_MODE_LAND;  
        	}
        });
        
        mGround=(Button) findViewById(R.id.btn_Ground);
        mGround.setOnClickListener(new OnClickListener() {
        	public void onClick(View v) {
//        		if(mThrustBar.getProgress()==0){
	        		mToolbarDisconnectButton.setVisibility(View.VISIBLE);
	        		mAssist.setVisibility(View.VISIBLE);
	        		mManual.setVisibility(View.VISIBLE);
	        		mLaunch.setVisibility(View.GONE);
	        		mControl.setVisibility(View.GONE);
	        		mLand.setVisibility(View.GONE);
	        		mGround.setVisibility(View.GONE);
	        		mThrustBar.setVisibility(View.GONE);
	    			mYawBar.setVisibility(View.GONE);
	    			mThrustTitle.setVisibility(View.GONE);
	    			mYawTitle.setVisibility(View.GONE);
	    			flightMode=FLIGHT_MODE_GROUND;  
//        		}
        	}
        });
//        mSendTextContainer = (View) findViewById(R.id.send_text_container);
        
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
        
      //Initialize the thrust bar with a listener for change events
        mThrustBar = (SeekBar) findViewById(R.id.ThrustBar);
        mThrustBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
        	public void onProgressChanged(SeekBar SeekBar, int progress, boolean fromUser) {
        	}

    		@Override
    		public void onStartTrackingTouch(SeekBar seekBar) {
    		}

    		@Override
    		public void onStopTrackingTouch(SeekBar seekBar) {
    			if(flightMode==FLIGHT_MODE_ASSIST){
    				mThrustBar.setProgress(50);
    			}
    		}
      	}); 
         
        //Initialize the thrust bar with a listener for change events
        mYawBar = (SeekBar) findViewById(R.id.YawBar);
        mYawBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
        	public void onProgressChanged(SeekBar SeekBar, int progress, boolean fromUser) {
        	}

    		@Override
    		public void onStartTrackingTouch(SeekBar seekBar) {
    		}

    		@Override
    		public void onStopTrackingTouch(SeekBar seekBar) {
    			mYawBar.setProgress(50);
    		}
      	});
        
        mYawBar.setProgress(50);
        
    
        
/*        mToolbarPauseButton = (ImageButton) findViewById(R.id.toolbar_btn_pause);
        mToolbarPauseButton.setOnClickListener(new OnClickListener(){
			public void onClick(View v) {
				paused = true;
				onPausedStateChanged();
			}
        });
*/        
/*        mToolbarPlayButton = (ImageButton) findViewById(R.id.toolbar_btn_play);
        mToolbarPlayButton.setOnClickListener(new OnClickListener(){
			public void onClick(View v) {
				paused = false;
				onPausedStateChanged();
			}
        });
*/        
        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        
        //Initialize Accelerometer
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
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

/*        // Initialize the array adapter for the conversation thread
        mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message);
        mConversationView = (ListView) findViewById(R.id.in);
        mConversationView.setAdapter(mConversationArrayAdapter);
        
        mConversationArrayAdapter.add("Welcome to QuadControl!");
        mConversationArrayAdapter.add("This application gives you a new way to control your quadcopter! ");
        mConversationArrayAdapter.add("Use the toolbar below to manage connections and get started.");
        mConversationArrayAdapter.add("You should receive a pairing request the first time the phone and command module are connected.  Make sure to accept this request so that the devices can communicate.");
        mConversationArrayAdapter.add("If the device ever fails to connect, reboot the command module and try again.");
*/
/*        // Initialize the compose field with a listener for the return key
        mOutEditText = (EditText) findViewById(R.id.edit_text_out);
        mOutEditText.setOnEditorActionListener(mWriteListener);

        // Initialize the send button with a listener that for click events
        mSendButton = (Button) findViewById(R.id.button_send);
        mSendButton.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                // Send a message using content of the edit text widget
                TextView view = (TextView) findViewById(R.id.edit_text_out);
                String message = view.getText().toString();
                sendMessage(message);
            }
        });
*/
       
        // Initialize the BluetoothService to perform Bluetooth connections
        mBluetoothService = new BluetoothChatService(mHandler);

        // Initialize the buffer for outgoing messages
//        mOutStringBuffer = new StringBuffer("");
        
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
     */
/*    private void sendMessage(String message) {
        // Check that we're actually connected before trying anything
        if (mBluetoothService.getState() != BluetoothChatService.STATE_CONNECTED) {
            Toast.makeText(this, R.string.not_connected, Toast.LENGTH_SHORT).show();
            return;
        }

        // Check that there's actually something to send
        if (message.length() > 0) {
        	message += "\n";
            // Get the message bytes and tell the BluetoothService to write
            byte[] send = message.getBytes();
            mBluetoothService.write(send);

            // Reset out string buffer to zero and clear the edit text field
            mOutStringBuffer.setLength(0);
            mOutEditText.setText(mOutStringBuffer);
        }
    }
*/    
    public static byte[] intToByteArray(int a ,int b, int c, int d, int e, int f, int g, int h, int i, int j, int k, int l, int m, int n, int o)
    {
        byte[] ret = new byte[15];
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
        ret[11] = (byte) (l & 0xFF);
        ret[12] = (byte) (m & 0xFF);
        ret[13] = (byte) (n & 0xFF);
        ret[14] = (byte) (o & 0xFF);
        
        return ret;
    }
    
    String ByteArraytoString(byte[] bytes) {
    	StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return(sb.toString());
   }

/*    // The action listener for the EditText widget, to listen for the return key
    private TextView.OnEditorActionListener mWriteListener =
        new TextView.OnEditorActionListener() {
        public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
            // If the action is a key-up event on the return key, send the message
            if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
                String message = view.getText().toString();
                sendMessage(message);
            }
            if(D) Log.i(TAG, "END onEditorAction");
            return true;
        }
    };
*/
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
                String writeMessage = new String(writeBuf);
//                mConversationArrayAdapter.add(">>> " + writeMessage);
                if (D) Log.d(TAG, "written = '" + ByteArraytoString(writeBuf) + "'");
                break;
            case MESSAGE_READ:
            	if (paused) break;
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
        	break;
        case R.id.menu_debug:
        	mA_Label.setVisibility(View.VISIBLE);
        	mA_Text.setVisibility(View.VISIBLE);
        	mA_Plus.setVisibility(View.VISIBLE);
        	mA_Minus.setVisibility(View.VISIBLE);
        	mB_Label.setVisibility(View.VISIBLE);
        	mB_Text.setVisibility(View.VISIBLE);
        	mB_Plus.setVisibility(View.VISIBLE);
        	mB_Minus.setVisibility(View.VISIBLE);
        	mC_Label.setVisibility(View.VISIBLE);
        	mC_Text.setVisibility(View.VISIBLE);
        	mC_Plus.setVisibility(View.VISIBLE);
        	mC_Minus.setVisibility(View.VISIBLE);
        	mD_Label.setVisibility(View.VISIBLE);
        	mD_Text.setVisibility(View.VISIBLE);
        	mD_Plus.setVisibility(View.VISIBLE);
        	mD_Minus.setVisibility(View.VISIBLE);
        	mp_Label.setVisibility(View.VISIBLE);
        	mp_Text.setVisibility(View.VISIBLE);
        	mp_Plus.setVisibility(View.VISIBLE);
        	mp_Minus.setVisibility(View.VISIBLE);
        	mi_Label.setVisibility(View.VISIBLE);
        	mi_Text.setVisibility(View.VISIBLE);
        	mi_Plus.setVisibility(View.VISIBLE);
        	mi_Minus.setVisibility(View.VISIBLE);
        	md_Label.setVisibility(View.VISIBLE);
        	md_Text.setVisibility(View.VISIBLE);
        	md_Plus.setVisibility(View.VISIBLE);
        	md_Minus.setVisibility(View.VISIBLE);
        	break;
        case R.id.menu_standard:
        	mA_Label.setVisibility(View.GONE);
        	mA_Text.setVisibility(View.GONE);
        	mA_Plus.setVisibility(View.GONE);
        	mA_Minus.setVisibility(View.GONE);
        	mB_Label.setVisibility(View.GONE);
        	mB_Text.setVisibility(View.GONE);
        	mB_Plus.setVisibility(View.GONE);
        	mB_Minus.setVisibility(View.GONE);
        	mC_Label.setVisibility(View.GONE);
        	mC_Text.setVisibility(View.GONE);
        	mC_Plus.setVisibility(View.GONE);
        	mC_Minus.setVisibility(View.GONE);
        	mD_Label.setVisibility(View.GONE);
        	mD_Text.setVisibility(View.GONE);
        	mD_Plus.setVisibility(View.GONE);
        	mD_Minus.setVisibility(View.GONE);
        	mp_Label.setVisibility(View.GONE);
        	mp_Text.setVisibility(View.GONE);
        	mp_Plus.setVisibility(View.GONE);
        	mp_Minus.setVisibility(View.GONE);
        	mi_Label.setVisibility(View.GONE);
        	mi_Text.setVisibility(View.GONE);
        	mi_Plus.setVisibility(View.GONE);
        	mi_Minus.setVisibility(View.GONE);
        	md_Label.setVisibility(View.GONE);
        	md_Text.setVisibility(View.GONE);
        	md_Plus.setVisibility(View.GONE);
        	md_Minus.setVisibility(View.GONE);
        	offsetA=0;
        	offsetB=0;
        	offsetC=0;
        	offsetD=0;
        	p=0;
        	i=0;
        	d=0;
        	break;
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
			mAssist.setVisibility(View.VISIBLE);
			mManual.setVisibility(View.VISIBLE);
//			mSendTextContainer.setVisibility(View.VISIBLE);
    	}
    	else {
			mToolbarConnectButton.setVisibility(View.VISIBLE);
			mToolbarDisconnectButton.setVisibility(View.GONE);
			mAssist.setVisibility(View.GONE);
    		mManual.setVisibility(View.GONE);
    		mLaunch.setVisibility(View.GONE);
    		mControl.setVisibility(View.GONE);
    		mLand.setVisibility(View.GONE);
    		mGround.setVisibility(View.GONE);
    		mThrustBar.setVisibility(View.GONE);
			mYawBar.setVisibility(View.GONE);
			mThrustTitle.setVisibility(View.GONE);
			mYawTitle.setVisibility(View.GONE);
			flightMode=FLIGHT_MODE_GROUND;  
			
//			mSendTextContainer.setVisibility(View.GONE);
    	}
		paused = false;
//    	onPausedStateChanged();
    }

/*    private void onPausedStateChanged() {
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
*/    
   
    @Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		double dox = 2*event.values[1]+90;
		int inx = (int)dox;
		double doy = 2*event.values[0]+90;
		int iny = (int)doy;
		if(inx<0){
			inx=0;
		}else if(inx>180){
			inx=180;
		}
		if(iny<0){
			iny=0;
		}else if(iny>180){
			iny=180;
		} 
		double power=207-2.05*mThrustBar.getProgress();
    	int throttle=(int)power;
    	int yaw = mYawBar.getProgress();
    	Log.d(TAG,""+inx+"     "+iny);
    	motorA=throttle-offsetA;
    	motorB=throttle-offsetB;
    	motorC=throttle-offsetC;
    	motorD=throttle-offsetD;
    	if(motorA<2){
    		offsetA=throttle-2;
    	}
    	if(motorB<2){
    		offsetB=throttle-2;
    	}
    	if(motorC<2){
    		offsetC=throttle-2;
    	}
    	if(motorD<2){
    		offsetD=throttle-2;
    	}
    	if(motorA>207){
    		offsetA=throttle-207;
    	}
    	if(motorB>207){
    		offsetB=throttle-207;
    	}
    	if(motorC>207){
    		offsetC=throttle-207;
    	}
    	if(motorD>207){
    		offsetD=throttle-207;
    	}
    	if(p<4){
    		p=4;
    	}
    	if(i<4){
    		i=4;
    	}
    	if(d<4){
    		d=4;
    	}
    	xdraw=360+inx;
    	ydraw=360+iny;
        mBluetoothService.write(intToByteArray(0xFA,0xEB,inx,iny,yaw,throttle,flightMode,p,i,d,motorA,motorB,motorC,motorD,0x0D));
        mThrustTitle.setText(""+mThrustBar.getProgress());
        if(mA_Label.getVisibility()==View.VISIBLE){
        	 mA_Text.setText(""+offsetA);
        	 mB_Text.setText(""+offsetB);
        	 mC_Text.setText(""+offsetC);
        	 mD_Text.setText(""+offsetD);
        	 mp_Text.setText(""+p);
        	 mi_Text.setText(""+i);
        	 md_Text.setText(""+d);
        }
        
//		  mdrawView = new DrawView(this,xdraw,ydraw);
        
//        mdrawView.setBackgroundColor(Color.TRANSPARENT);
//        setContentView(mdrawView);
	}    
}