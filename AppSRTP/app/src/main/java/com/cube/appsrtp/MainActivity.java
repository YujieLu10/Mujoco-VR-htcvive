package com.cube.appsrtp;

import android.content.Intent;
import android.content.res.AssetManager;
import android.graphics.Bitmap;

import android.net.Uri;
import android.os.Bundle;
import android.provider.MediaStore;
import android.support.v7.app.AppCompatActivity;

import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import com.cube.tesscv.tesscv;

import java.io.IOException;
import java.io.InputStream;

import org.opencv.android.OpenCVLoader;


public class MainActivity extends AppCompatActivity {
    public static final String IMAGE_UNSPECIFIED = "image/*";
    public static final int PHOTOALBUM = 1;   // 相册
    Button photo_album = null;                // 相册
    ImageView imageView = null;               // 截取图像
    TextView textView = null;                 // OCR 识别结果

    Bitmap m_phone;                           // Bitmap图像
    String m_ocrOfBitmap;                     // Bitmap图像OCR识别结果
    InputStream m_instream;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        imageView = (ImageView) findViewById(R.id.imageID);
        photo_album = (Button) findViewById(R.id.photo_album);
        textView = (TextView) findViewById(R.id.OCRTextView);
        photo_album.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(Intent.ACTION_PICK, null);
                intent.setDataAndType(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, IMAGE_UNSPECIFIED);
                startActivityForResult(intent, PHOTOALBUM);
            }
        });

        //get access to AssetManager
        AssetManager assetManager = getAssets();
        //open byte streams for reading/writing
        try {
            m_instream = assetManager.open("tessdata/eng.traineddata");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == 0 || data == null) {
            return;
        }
        // 相册
        if (requestCode == PHOTOALBUM) {
            Uri image = data.getData();
            try {
                m_phone = MediaStore.Images.Media.getBitmap(getContentResolver(), image);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        // 处理结果
        imageView.setImageBitmap(m_phone);
        if (OpenCVLoader.initDebug()) {
            // do some opencv stuff
            tesscv jmi = new tesscv(m_phone, m_instream);
            m_ocrOfBitmap = jmi.getOcrOfBitmap();
        }
        textView.setText(m_ocrOfBitmap);
        super.onActivityResult(requestCode, resultCode, data);
    }
}