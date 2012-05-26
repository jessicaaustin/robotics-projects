package org.aus10.iohannes;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends Activity {
    private TextView title_;
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

        findViewById(R.id.motor_forward).setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                moveMotors(IOIOService.MotorControl.FORWARD, view);
            }
        });
        findViewById(R.id.motor_backward).setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                moveMotors(IOIOService.MotorControl.REVERSE, view);
            }
        });
        findViewById(R.id.motor_left).setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                moveMotors(IOIOService.MotorControl.LEFT, view);
            }
        });
        findViewById(R.id.motor_right).setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                moveMotors(IOIOService.MotorControl.RIGHT, view);
            }
        });
    }

    private void moveMotors(IOIOService.MotorControl direction, View view) {
        sendBroadcast(IOIOService.createIntentToMoveMotor(direction));
        startActivityForResult(new Intent(view.getContext(), StopActivity.class), 0);
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
