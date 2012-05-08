package org.aus10.iohannes;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;
import android.widget.ToggleButton;

public class MainActivity extends Activity {
    private TextView title_;
    private ToggleButton motorForwardButton;
    private ToggleButton motorReverseButton;
    private ToggleButton motorLeftButton;
    private ToggleButton motorRightButton;
    private TextView ultrasoundDistance;
    public static final String INTENT_CONNECTED = "ioioConnected";
    public static final String INTENT_PARAM_CONNECTED = "status";
    public static final String INTENT_PARAM_OBSTACLE_DISTANCE = "obstacleDistanceInCm";

    private final IntentFilter intentFilter = new IntentFilter(INTENT_CONNECTED);
    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String stringExtra = intent.getStringExtra(INTENT_PARAM_CONNECTED);
            if (stringExtra != null) {
                title_.setText(stringExtra);
            }
            int obstacleDistanceInCm = intent.getIntExtra(INTENT_PARAM_OBSTACLE_DISTANCE, -1);
            if (obstacleDistanceInCm != -1) {
                ultrasoundDistance.setText(Integer.toString(obstacleDistanceInCm));
            }

        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        title_ = (TextView) findViewById(R.id.title);
        ultrasoundDistance = (TextView) findViewById(R.id.ultrasound_distance);
        registerReceiver(broadcastReceiver, intentFilter);
        motorForwardButton = (ToggleButton) findViewById(R.id.motor_forward);
        motorReverseButton = (ToggleButton) findViewById(R.id.motor_backward);
        motorLeftButton = (ToggleButton) findViewById(R.id.motor_left);
        motorRightButton = (ToggleButton) findViewById(R.id.motor_right);
        motorForwardButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                sendBroadcast(MotorService.createIntentToMoveMotor(MotorService.MotorControl.FORWARD));
                motorReverseButton.setChecked(false);
                motorLeftButton.setChecked(false);
                motorRightButton.setChecked(false);
            }
        });
        motorReverseButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                sendBroadcast(MotorService.createIntentToMoveMotor(MotorService.MotorControl.REVERSE));
                motorForwardButton.setChecked(false);
                motorLeftButton.setChecked(false);
                motorRightButton.setChecked(false);
            }
        });
        motorLeftButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                sendBroadcast(MotorService.createIntentToMoveMotor(MotorService.MotorControl.LEFT));
                motorForwardButton.setChecked(false);
                motorReverseButton.setChecked(false);
                motorRightButton.setChecked(false);
            }
        });
        motorRightButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                sendBroadcast(MotorService.createIntentToMoveMotor(MotorService.MotorControl.RIGHT));
                motorForwardButton.setChecked(false);
                motorReverseButton.setChecked(false);
                motorLeftButton.setChecked(false);
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();

    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(broadcastReceiver);
    }
}
