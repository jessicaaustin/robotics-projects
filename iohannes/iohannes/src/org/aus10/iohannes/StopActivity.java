package org.aus10.iohannes;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

public class StopActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.stop);

        findViewById(R.id.stop).setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                sendBroadcast(IOIOService.createIntentToMoveMotor(IOIOService.MotorControl.STOP));
                startActivityForResult(new Intent(view.getContext(), MainActivity.class), 0);
            }
        });
    }
}
