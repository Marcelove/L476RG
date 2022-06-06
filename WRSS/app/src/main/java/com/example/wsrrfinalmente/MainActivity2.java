package com.example.wsrrfinalmente;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

import java.util.Timer;
import java.util.TimerTask;


public class MainActivity2 extends AppCompatActivity {
    FirebaseDatabase database = FirebaseDatabase.getInstance();

    Timer timer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main2);
        timer = new Timer();

        DatabaseReference open = database.getReference("Open");
        open.setValue(1);

        Handler handler = new Handler();

        handler.postDelayed(new Runnable() {
            public void run() {
                open.setValue(0);
                finish();
            }
        }, 17000);

    }

}