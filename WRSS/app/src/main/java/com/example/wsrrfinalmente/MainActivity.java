package com.example.wsrrfinalmente;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import com.google.android.material.snackbar.Snackbar;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.util.Timer;

public class MainActivity extends AppCompatActivity {

    EditText inputText;
    EditText inputSenha;
    private static final String TAG = "test";
    FirebaseDatabase database = FirebaseDatabase.getInstance();

    Timer timer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        inputSenha = (EditText) findViewById(R.id.senha);
        inputText = (EditText) findViewById(R.id.text);
        Button button = (Button) findViewById(R.id.button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                String nome = inputText.getText().toString();
                String senha = inputSenha.getText().toString();
                int cont = 1;
                while(cont<=2){
                    String s=Integer.toString(cont);
                    DatabaseReference myRef = database.getReference("/Pessoa" + s);
                    myRef.addValueEventListener(new ValueEventListener() {
                        @Override
                        public void onDataChange(DataSnapshot dataSnapshot) {
                            String nome_firebase = dataSnapshot.child("Nome").getValue(String.class);
                            String senha_firebase = dataSnapshot.child("Senha").getValue(String.class);
                            if(nome.equals(nome_firebase) && senha.equals(senha_firebase)){
                                DatabaseReference open = database.getReference("Open");
                                startActivity(new Intent(MainActivity.this, MainActivity2.class));
                            }
                            else{
                                Snackbar.make(v,"ERRO: Login ou senha inválidos.", Snackbar.LENGTH_LONG)
                                        .setAction("Action", null).show();
                            }
                        }
                        @Override
                        public void onCancelled(DatabaseError error) {
                            Log.w(TAG, "Failed to read value.", error.toException());
                        }
                    });
                    cont++;
                }
            }
        });
    }
}