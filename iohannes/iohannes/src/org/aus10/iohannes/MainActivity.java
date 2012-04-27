package org.aus10.iohannes;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.widget.TextView;

public class MainActivity extends Activity {
    private TextView title_;
    public static final String INTENT_CONNECTED = "ioioConnected";
    public static final String INTENT_PARAM_CONNECTED = "status";
    private final IntentFilter intentFilter = new IntentFilter(INTENT_CONNECTED);

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String stringExtra = intent.getStringExtra(INTENT_PARAM_CONNECTED);
            title_.setText(stringExtra);
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        title_ = (TextView) findViewById(R.id.title);
        registerReceiver(broadcastReceiver, intentFilter);
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
