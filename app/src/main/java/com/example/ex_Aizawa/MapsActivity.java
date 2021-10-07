package com.example.ex_Aizawa;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.LocationProvider;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.location.LocationResult;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.UiSettings;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PolylineOptions;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;
import java.util.zip.DeflaterInputStream;

import static android.hardware.SensorManager.SENSOR_DELAY_NORMAL;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback,LocationListener,GpsActivity.OnLocationResultListener {

    //Bluetooth Adapter
    private BluetoothAdapter mAdapter;
    private BluetoothDevice mDevice;
    private final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");  //なんでもいい臭いけど要調査
    private BluetoothSocket mSocket;
    OutputStream mmOutputStream = null;
    InputStream mmInStream = null;
    boolean connectFlg = false;                  //盲導盤との接続状態
    String output = null;                        //盲導盤への指令　節電用に用いる

    //Googlemap関連
    private GoogleMap mMap = null;
    private LocationManager locationManager;
    private GpsActivity gpsActivity;

    //テキスト
    public static TextView bluetoothState;      //bluetoothの状態表示
    public static TextView gpsState;            //GPSの状態
    //public TextView current_Lat;                //緯度
    //public TextView current_Lng;                //経度
    public TextView targetPoint;
    public TextView currentPoint;
    public TextView status;                     //誘導状況表示
    public static TextView Mag;                 //方位

    private static final String PROVIDER_ENABLED = "GPS enabled";
    private static final String PROVIDER_DISABLED = "GPS disabled";

    //ボタン
    private Button connectBt;
    private Button saveBt;
    private Button startBt;
    private Button stopBt;
    private Button modeBt;
    private Button upBt;
    private Button downBt;
    private Button rightBt;
    private Button leftBt;

    double currentLat;
    double currentLng;

    double FcurrentLat = 37.90106444;//シミュレーション用
    double FcurrentLng = 140.1054401;

    //実験用緯度経度座標
    public static MarkerOptions options;
    double startLat;                 //案内開始時のスタート緯度　
    double startLng;                 //案内開始時のスタート経度
    double exLat = 37.901064444656;  //ex1の緯度
    double exLng = 140.105440123665;  //ex1の経度
    double addLat = 0.00004505;  //5[m]分の移動量(緯度換算)
    double addLng = 0.00005685;  //5[m]分の移動量(経度換算)
    //double addLat = 0.00001802;  //2[m]分の移動量(緯度換算)
    //double addLng = 0.00002274;  //2[m]分の移動量(経度換算)
    double ex2Lat = exLat + addLat/5;
    double ex3Lat = exLat + addLat/2;
    double ex4Lat = exLat - addLat/5;
    double ex5Lat = exLat - addLat/2;
    double ex2Lng = exLng - addLng*3/10;
    double ex3Lng = exLng - addLng*7/10;
    double ex4Lng = exLng - addLng;
    double ex5Lng = exLng + addLng*3/10;
    double ex6Lng = exLng + addLng*7/10;
    double ex7Lng = exLng + addLng;

    LatLng ex1 = new LatLng(exLat, exLng);
    LatLng ex2 = new LatLng(ex2Lat, exLng);
    LatLng ex3 = new LatLng(ex3Lat, ex2Lng);
    LatLng ex4 = new LatLng(ex3Lat, ex3Lng);
    LatLng ex5 = new LatLng(ex2Lat, ex4Lng);
    LatLng ex6 = new LatLng(ex4Lat, ex4Lng);
    LatLng ex7 = new LatLng(ex5Lat, ex3Lng);
    LatLng ex8 = new LatLng(ex5Lat, ex2Lng);
    LatLng ex9 = new LatLng(ex4Lat, exLng);
    LatLng ex10 = new LatLng(ex2Lat, exLng);
    LatLng ex11 = new LatLng(ex3Lat, ex5Lng);
    LatLng ex12 = new LatLng(ex3Lat, ex6Lng);
    LatLng ex13 = new LatLng(ex2Lat, ex7Lng);
    LatLng ex14 = new LatLng(ex4Lat, ex7Lng);
    LatLng ex15 = new LatLng(ex5Lat, ex6Lng);
    LatLng ex16 = new LatLng(ex5Lat, ex5Lng);
    LatLng ex17 = new LatLng(ex4Lat, exLng);

    LatLng guidePoint;

    private float[] results = new float[3];
    private float[] results1 = new float[3];
    private float[] results2 = new float[3];
    private float[] results3 = new float[3];
    private float[] results4 = new float[3];
    private float[] results5 = new float[3];//GPSによる2点間の距離，角度
    int waypointDeg;
    double maxDistance = 1.75;
    double objectDistance;
    double closestDistance;
    double guideDistance;

    int path_val = 19;
    int mode = 1;
    double[] pathLat = new double[19];
    double[] pathLng = new double[19];

    int startCount = 0;

    int MDeg = 0;
    int SDeg = 0;
    int DegOffSet = 0;

    //タイマー関連
    private Timer mainTimer;
    private MainTimerTask mainTimerTask = null;
    private Handler timerHandler = new Handler();
    double dt = 1000;

    //Google Maps関連
    //private GoogleMap mMap;
    //public String travelMode = "walking";  //default  ここでルート検索の際歩行を優先したルートが表示される
    //ArrayList<LatLng> markerPoints;

    //GpsActivityのインスタンス生成
    //private GpsActivity gpsActivity;

    //加速度・地磁気センサ関連
    public SensorManager sensorManager;
    Sensor s1, s2;
    sensorChangeEvent sensorChangeEvent;


    //保存用
    int val = 30000;
    double[] array1 = new double[val];
    double[] array2 = new double[val];
    double[] array3 = new double[val];
    double[] array4 = new double[val];
    double[] array5 = new double[val];
    double[] array6 = new double[val];
    double[] array7 = new double[val];
    double[] array8 = new double[val];
    double time_count = 0;
    String text;
    int measure_val = 0;

    private int target_deg;                     //誘導の際の目標角度

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        //描画設定
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);

        //テキスト表示設定
        bluetoothState = findViewById(R.id.bluetoothState);
        gpsState = findViewById(R.id.gps_state);
        targetPoint = findViewById(R.id.target);
        currentPoint = findViewById(R.id.now);
        status = (findViewById(R.id.status));
        status.setBackgroundColor(0xffffffff);
        Mag = findViewById(R.id.mag);

        //ボタン初期化
        connectBt = findViewById(R.id.connect_bt);
        connectBt.setOnClickListener(new ClickEvent());

        saveBt = findViewById(R.id.save_bt);
        saveBt.setOnClickListener(new ClickEvent());

        startBt = findViewById(R.id.start_bt);
        startBt.setOnClickListener(new ClickEvent());

        stopBt = findViewById(R.id.stop_bt);
        stopBt.setOnClickListener(new ClickEvent());

        modeBt = findViewById(R.id.mode_bt);
        modeBt.setOnClickListener(new ClickEvent());

        upBt = findViewById(R.id.up_bt);
        upBt.setOnClickListener(new ClickEvent());

        downBt = findViewById(R.id.down_bt);
        downBt.setOnClickListener(new ClickEvent());

        rightBt = findViewById(R.id.right_bt);
        rightBt.setOnClickListener(new ClickEvent());

        leftBt = findViewById(R.id.left_bt);
        leftBt.setOnClickListener(new ClickEvent());


        //GPS関係
        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        gpsActivity = new GpsActivity(this ,this);
       /* boolean GPSFlg = locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
        String GPSstatus = GPSFlg ? PROVIDER_ENABLED : PROVIDER_DISABLED;
        gpsState.setText(GPSstatus);*/


        //GoogleMaps設定
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        //センサ
        sensorChangeEvent = new sensorChangeEvent();

        //Bluetooth設定
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        bluetoothState.setText(R.string.search_device);
        Set<BluetoothDevice> devices = mAdapter.getBondedDevices();
        for (BluetoothDevice device : devices) {
            //string型の固定値の比較.equals(string)
            //ペアリング用　盲導盤の回路に搭載されてるbluetoothモジュール参照
            //String DEVICE_NAME = "SBDBT-001bdc087049";
            String DEVICE_NAME = "SBDBT-001bdc08c1e0";
            if (device.getName().equals(DEVICE_NAME)) {

                bluetoothState.setText(device.getName());
                mDevice = device;
            }
        }
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
    }

    public static String GetNowDate() {

        Calendar calendar = Calendar.getInstance();
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat("MMdd_HH:mm:ss");
        String Date = simpleDateFormat.format(calendar.getTime());
        return Date;
    }

    //アプリ立ち上げ時
    protected void onResume() {
        super.onResume();

        /*if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }*/
        //locationManager.requestLocationUpdates(LocationManager.PASSIVE_PROVIDER, 1000, 0, this);
        gpsActivity.startLocationUpdates();

        //加速度センサ、地磁気センサ起動
        s1 = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        s2 = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        sensorManager.registerListener(sensorChangeEvent, s1, SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(sensorChangeEvent, s2, SENSOR_DELAY_NORMAL);

    }

    //アプリ終了時
    protected void onPause() {
        super.onPause();
        //GPS終了
        if (locationManager != null) {
            if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                    && ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                return;
            }
            locationManager.removeUpdates(this);
        }

        if (gpsActivity != null) {
            gpsActivity.stopLocationUpdates();
        }

        //加速度センサ、地磁気センサ終了
        sensorManager.unregisterListener(sensorChangeEvent, s1);
        sensorManager.unregisterListener(sensorChangeEvent, s2);
        //bluetooth接続終了
        try {
            mSocket.close();
        } catch (Exception e) {
        }
    }

    public static double deg2rad(double deg) {
        return deg * Math.PI / 180.0;
    }

    public static double getDistance(double lat1, double lng1, double lat2, double lng2) {
        double my = deg2rad((lat1 + lat2) / 2.0);
        double dy = deg2rad(lat1 - lat2);
        double dx = deg2rad(lng1 - lng2);

        double Rx = 6378137.000;
        double Ry = 6356752.314245;
        double E = Math.sqrt((Rx * Rx - Ry * Ry) / (Rx * Rx));

        double sin = Math.sin(my);
        double cos = Math.cos(my);
        double W = Math.sqrt(1.0 - E * E * sin * sin);
        double M = Rx * (1 - E * E) / (W * W * W);
        double N = Rx / W;

        double dym = dy * M;
        double dxncos = dx * N * cos;

        return Math.sqrt(dym * dym + dxncos * dxncos);
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        //実験やる場所映す
        mMap = googleMap;
        LatLng firstPosition = new LatLng(37.901064444656, 140.105440123665);
        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(firstPosition, 18));

        UiSettings uiSettings = mMap.getUiSettings();
        mMap.setMyLocationEnabled(true);                     //現在位置の丸を表示するためのもの
        uiSettings.setZoomControlsEnabled(true);             //ズームとかするためのボタン出すやつ
        uiSettings.setMyLocationButtonEnabled(true);         //現在位置に飛ぶためのボタン表示のためのやつ
        marker();  //実験用のマーカー表示
    }

    //実験用のマーカー表示
    void marker() {
        options = new MarkerOptions();
        options.position(ex1);
        mMap.addMarker(options);
        options.position(ex2);
        mMap.addMarker(options);
        options.position(ex3);
        mMap.addMarker(options);
        options.position(ex4);
        mMap.addMarker(options);
        options.position(ex5);
        mMap.addMarker(options);
        options.position(ex6);
        mMap.addMarker(options);
        options.position(ex7);
        mMap.addMarker(options);
        options.position(ex8);
        mMap.addMarker(options);
        options.position(ex9);
        mMap.addMarker(options);
        options.position(ex10);
        mMap.addMarker(options);
        options.position(ex11);
        mMap.addMarker(options);
        options.position(ex12);
        mMap.addMarker(options);
        options.position(ex13);
        mMap.addMarker(options);
        options.position(ex14);
        mMap.addMarker(options);
        options.position(ex15);
        mMap.addMarker(options);
        options.position(ex16);
        mMap.addMarker(options);
        options.position(ex17);
        mMap.addMarker(options);
    }

    //GPSによる割り込み　ここから
    //GPSによって位置情報が変化した際のイベント
    public void onLocationResult(LocationResult locationResult) {
        if (locationResult == null) {
            return;
        }
        // 緯度・経度を取得
        currentLat = locationResult.getLastLocation().getLatitude();
        currentLng = locationResult.getLastLocation().getLongitude();
    }

    //GPSによって位置情報が変化した際のイベント
    @Override
    public void onLocationChanged(Location location) {

    }

    public static LatLng computeOffset(LatLng from, double distance, double heading) {
        distance /= 6371009.0D;  //earth_radius = 6371009 # in meters
        heading = Math.toRadians(heading);
        double fromLat = Math.toRadians(from.latitude);
        double fromLng = Math.toRadians(from.longitude);
        double cosDistance = Math.cos(distance);
        double sinDistance = Math.sin(distance);
        double sinFromLat = Math.sin(fromLat);
        double cosFromLat = Math.cos(fromLat);
        double sinLat = cosDistance * sinFromLat + sinDistance * cosFromLat * Math.cos(heading);
        double dLng = Math.atan2(sinDistance * cosFromLat * Math.sin(heading), cosDistance - sinFromLat * sinLat);
        return new LatLng(Math.toDegrees(Math.asin(sinLat)), Math.toDegrees(fromLng + dLng));
    }

    /*public void onLocationChanged(Location location) {
        currentLat = location.getLatitude();
        currentLng = location.getLongitude();
        // 緯度・経度を取得
        //String Lat = "currentLat:" + currentLat;
        //String Lng = "currentLng" + currentLng;
    }*/

    //クリックしたときのイベント
    class ClickEvent implements View.OnClickListener {
        public void onClick(View v) {
            if (v.equals(connectBt)) connect();   //bluetooth接続ボタン
            else if (v.equals(saveBt)) save();    //記録用ボタン
            else if (v.equals(startBt)) start();  //誘導開始用ボタン
            else if (v.equals(stopBt)) stop();    //誘導強制終了用ボタン
            else if (v.equals(modeBt)) mode();  //音声入力用ボタン
            else if (v.equals(upBt)) up();
            else if (v.equals(downBt)) down();
            else if (v.equals(rightBt)) right();
            else if (v.equals(leftBt)) left();
        }


        private void connect() {
            if (!connectFlg) {
                try {
                    // 取得したデバイス名を使ってBluetoothでSocket接続
                    mSocket = mDevice.createRfcommSocketToServiceRecord(MY_UUID);
                    mSocket.connect();
                    mmInStream = mSocket.getInputStream();
                    mmOutputStream = mSocket.getOutputStream();
                    bluetoothState.setText(R.string.connected);
                    mmOutputStream.write("START#".getBytes());
                    connectFlg = true;
                    DegOffSet = (int) sensorChangeEvent.Deg;
                    if(DegOffSet < 0){
                        DegOffSet = DegOffSet + 360;
                    }
                    DegOffSet = DegOffSet / 2;
                    mmOutputStream.write(DegOffSet);
                    DegOffSet = DegOffSet*2;
                    if(DegOffSet > 180){
                        DegOffSet = DegOffSet -360;
                    }
                } catch (Exception e) {
                    bluetoothState.setText((CharSequence) e);
                    try {
                        mSocket.close();
                    } catch (Exception ee) {
                        connectFlg = false;
                    }
                }

            }

        }

        private void save() {
            time_count = 0;
            // [デバイス・ファイル・エクスプローラー]/date/date/[アプリ名]/filesで確認可
            String getNowDate = GetNowDate();
            //String path = getFilesDir()+"/" + getNowDate +".txt";
            //String[] paths = {getFilesDir()+"/" + getNowDate + ".txt"};
            // [デバイス・ファイル・エクスプローラー]/sdcard/Android/data/[アプリ名]/files/documentsで確認可
            //andoroid実機では　設定/ストレージ/ファイルから上と同じように確認可能
            String path = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS) + "/" + getNowDate +".txt";
            String[] paths = {getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS) + "/" + getNowDate + ".txt"};
            String[] mimeTypes = {"text/plain"};

            try{ //ここが変わった

                FileOutputStream fileOutputStream = new FileOutputStream(path);
                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(fileOutputStream);
                BufferedWriter bufferedWriter = new BufferedWriter(outputStreamWriter);

                //実際に歩いた緯度経度を記録------------------------------------------------------------------------------------------------
/*                text = 0 + "\t" + (String.format("%.8f",exLng)) +  "\t" + (String.format("%.8f",exLat));
                bufferedWriter.write(text);
                bufferedWriter.newLine();
                text = 0 + "\t" + (String.format("%.8f",ex2Lng)) + "\t" + (String.format("%.8f",ex2Lat));
                bufferedWriter.write(text);
                bufferedWriter.newLine();
                text = 0 + "\t" + (String.format("%.8f",ex3Lng)) + "\t" + (String.format("%.8f",ex3Lat));
                bufferedWriter.write(text);
                bufferedWriter.newLine();
                text = ("---------------------------------------------------------------------------");
                bufferedWriter.write(text);
                bufferedWriter.newLine();
*/
                text = "time" + "\t" + "mode" + "\t" + "Lng" + "\t" + "Lat" + "\t"  + "MoudobanDeg" + "\t"  + "SmartPhoneDeg" + "\t" + "TargetDeg"+ "\t" + "TargetLng"+ "\t" + "TargetLat";
                bufferedWriter.write(text);
                bufferedWriter.newLine();
                for (int storage_val = 0; storage_val < measure_val; storage_val++) {
                    text = time_count/1000 + "\t" + array6[storage_val] + "\t" + (String.format("%.6f",array2[storage_val])) + "\t" + (String.format("%.6f",array1[storage_val]))
                            + "\t" + (String.format("%.0f",array3[storage_val]))+ "\t" + (String.format("%.0f",array4[storage_val]))+ "\t" + (String.format("%.0f",array5[storage_val]))
                            + "\t" + (String.format("%.6f",array7[storage_val]))+ "\t" + (String.format("%.6f",array8[storage_val]));
                    bufferedWriter.write(text);
                    bufferedWriter.newLine();
                    time_count += dt;
                }
                bufferedWriter.flush();
                bufferedWriter.close();
                Toast.makeText(MapsActivity.this, "Saved data.", Toast.LENGTH_SHORT).show();
            } catch (IOException e) {
                e.printStackTrace();

            }
            MediaScannerConnection.scanFile(getApplicationContext(), paths, mimeTypes, new MediaScannerConnection.OnScanCompletedListener() {
                public void onScanCompleted(String path, Uri uri) {
                }
            });
        }

        private void start() {
            startCount = 1;
            if (null != mainTimer) {
                mainTimer.cancel();
                mainTimer = null;
                mainTimerTask = null;
            }
            mMap.clear();
            path_val = 0;
            measure_val = 0;
            startLat = currentLat;    //現在地
            startLng = currentLng;
            /*if (currentLat != 0 && currentLng != 0) {
                exLat = currentLat;
                exLng = currentLng;
            }*/
            experiment_mode(mode);  //各モードでのルート設定
            marker();
            //compass();
            //計測＆誘導開始
            Toast.makeText(MapsActivity.this, "Start measurement.", Toast.LENGTH_SHORT).show();
            mainTimer = new Timer();
            mainTimerTask = new MainTimerTask();
            mainTimer.schedule(mainTimerTask, 0, (int) dt);    //1000[ms]間隔
            output = "0";

            FcurrentLat = 37.901064444656;
            FcurrentLng = 140.105440123665;
        }

        private void stop() {
            if (null != mainTimer) {
                startCount = 0;
                currentLat = pathLat[path_val];
                currentLng = pathLng[path_val];
                mainTimer = new Timer();
                mainTimer.cancel();
                mainTimer = null;
                mainTimerTask = null;
            }
        }

        private void mode() {
            if (mode == 3) mode = 1;
            else mode++;
            status.setText("実験モード" + mode + "に変更");
        }

        private void up() {
            FcurrentLat = FcurrentLat + addLat/10;
        }

        private void down() {
            FcurrentLat = FcurrentLat - addLat/10;
        }

        private void right() {
            FcurrentLng = FcurrentLng + addLng/10;
        }

        private void left() {
            FcurrentLng = FcurrentLng - addLng/10;
        }

    }

    //各モードでのルート設定
    void experiment_mode(int mode) {
        ex2Lat = exLat + addLat/5;
        ex3Lat = exLat + addLat/2;
        ex4Lat = exLat - addLat/5;
        ex5Lat = exLat - addLat/2;
        ex2Lng = exLng - addLng*3/10;
        ex3Lng = exLng - addLng*7/10;
        ex4Lng = exLng - addLng;
        ex5Lng = exLng + addLng*3/10;
        ex6Lng = exLng + addLng*7/10;
        ex7Lng = exLng + addLng;
        switch (mode) {
            case 1:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = ex2Lat;
                pathLng[1] = exLng;   //ex4
                pathLat[2] = ex3Lat;
                pathLng[2] = ex2Lng;   //ex7
                pathLat[3] = ex3Lat;
                pathLng[3] = ex3Lng;  //ex8
                pathLat[4] = ex2Lat;
                pathLng[4] = ex4Lng;  //ex9
                pathLat[5] = ex4Lat;
                pathLng[5] = ex4Lng;  //ex6
                pathLat[6] = ex5Lat;
                pathLng[6] = ex3Lng;  //ex3
                pathLat[7] = ex5Lat;
                pathLng[7] = ex2Lng;  //ex2
                pathLat[8] = ex4Lat;
                pathLng[8] = exLng;   //ex1
                pathLat[9] = ex2Lat;
                pathLng[9] = exLng;   //ex1
                pathLat[10] = ex3Lat;
                pathLng[10] = ex5Lng;   //ex4
                pathLat[11] = ex3Lat;
                pathLng[11] = ex6Lng;   //ex7
                pathLat[12] = ex2Lat;
                pathLng[12] = ex7Lng;  //ex8
                pathLat[13] = ex4Lat;
                pathLng[13] = ex7Lng;  //ex9
                pathLat[14] = ex5Lat;
                pathLng[14] = ex6Lng;  //ex6
                pathLat[15] = ex5Lat;
                pathLng[15] = ex5Lng;  //ex3
                pathLat[16] = ex4Lat;
                pathLng[16] = exLng;  //ex2
                pathLat[17] = exLat;
                pathLng[17] = exLng;   //ex1
                break;

            case 2:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = ex2Lat;
                pathLng[1] = exLng;   //ex4
                pathLat[2] = ex3Lat;
                pathLng[2] = ex2Lng;   //ex7
                pathLat[3] = ex3Lat;
                pathLng[3] = ex3Lng;  //ex8
                pathLat[4] = ex2Lat;
                pathLng[4] = ex4Lng;  //ex9
                pathLat[5] = ex4Lat;
                pathLng[5] = ex4Lng;  //ex6
                pathLat[6] = ex5Lat;
                pathLng[6] = ex3Lng;  //ex3
                pathLat[7] = ex5Lat;
                pathLng[7] = ex2Lng;  //ex2
                pathLat[8] = ex4Lat;
                pathLng[8] = exLng;   //ex1
                pathLat[9] = ex2Lat;
                pathLng[9] = exLng;   //ex1
                pathLat[10] = ex3Lat;
                pathLng[10] = ex5Lng;   //ex4
                pathLat[11] = ex3Lat;
                pathLng[11] = ex6Lng;   //ex7
                pathLat[12] = ex2Lat;
                pathLng[12] = ex7Lng;  //ex8
                pathLat[13] = ex4Lat;
                pathLng[13] = ex7Lng;  //ex9
                pathLat[14] = ex5Lat;
                pathLng[14] = ex6Lng;  //ex6
                pathLat[15] = ex5Lat;
                pathLng[15] = ex5Lng;  //ex3
                pathLat[16] = ex4Lat;
                pathLng[16] = exLng;  //ex2
                pathLat[17] = exLat;
                pathLng[17] = exLng;   //ex1
                break;

/*            case 2:
                pathLat[0] = ex2Lat;
                pathLng[0] = ex4Lng;  //ex9
                pathLat[1] = ex4Lat;
                pathLng[1] = ex4Lng;  //ex6
                pathLat[2] = ex5Lat;
                pathLng[2] = ex3Lng;  //ex3
                pathLat[3] = ex5Lat;
                pathLng[3] = ex2Lng;  //ex2
                pathLat[4] = ex4Lat;
                pathLng[4] = exLng;   //ex1
                pathLat[5] = ex2Lat;
                pathLng[5] = exLng;   //ex1
                pathLat[6] = ex3Lat;
                pathLng[6] = ex5Lng;   //ex4
                pathLat[7] = ex3Lat;
                pathLng[7] = ex6Lng;   //ex7
                pathLat[8] = ex2Lat;
                pathLng[8] = ex7Lng;  //ex8
                pathLat[9] = ex4Lat;
                pathLng[9] = ex7Lng;  //ex9
                pathLat[10] = ex5Lat;
                pathLng[10] = ex6Lng;  //ex6
                pathLat[11] = ex5Lat;
                pathLng[11] = ex5Lng;  //ex3
                pathLat[12] = ex4Lat;
                pathLng[12] = exLng;  //ex2
                pathLat[13] = exLat;
                pathLng[13] = exLng;   //ex1
                pathLat[14] = ex2Lat;
                pathLng[14] = exLng;   //ex4
                pathLat[15] = ex3Lat;
                pathLng[15] = ex2Lng;   //ex7
                pathLat[16] = ex3Lat;
                pathLng[16] = ex3Lng;  //ex8
                pathLat[17] = ex2Lat;
                pathLng[17] = ex4Lng;  //ex9
                break;

            case 3:
                pathLat[0] = ex2Lat;
                pathLng[0] = ex7Lng;  //ex8
                pathLat[1] = ex4Lat;
                pathLng[1] = ex7Lng;  //ex9
                pathLat[2] = ex5Lat;
                pathLng[2] = ex6Lng;  //ex6
                pathLat[3] = ex5Lat;
                pathLng[3] = ex5Lng;  //ex3
                pathLat[4] = ex4Lat;
                pathLng[4] = exLng;  //ex2
                pathLat[5] = exLat;
                pathLng[5] = exLng;   //ex1
                pathLat[6] = ex2Lat;
                pathLng[6] = exLng;   //ex4
                pathLat[7] = ex3Lat;
                pathLng[7] = ex2Lng;   //ex7
                pathLat[8] = ex3Lat;
                pathLng[8] = ex3Lng;  //ex8
                pathLat[9] = ex2Lat;
                pathLng[9] = ex4Lng;  //ex9
                pathLat[10] = ex4Lat;
                pathLng[10] = ex4Lng;  //ex6
                pathLat[11] = ex5Lat;
                pathLng[11] = ex3Lng;  //ex3
                pathLat[12] = ex5Lat;
                pathLng[12] = ex2Lng;  //ex2
                pathLat[13] = ex4Lat;
                pathLng[13] = exLng;   //ex1
                pathLat[14] = ex2Lat;
                pathLng[14] = exLng;   //ex1
                pathLat[15] = ex3Lat;
                pathLng[15] = ex5Lng;   //ex4
                pathLat[16] = ex3Lat;
                pathLng[16] = ex6Lng;   //ex7
                pathLat[17] = ex2Lat;
                pathLng[17] = ex7Lng;  //ex8
                break;

            case 4:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = exLat;
                pathLng[1] = ex2Lng;  //ex2
                pathLat[2] = exLat;
                pathLng[2] = ex3Lng;  //ex3
                pathLat[3] = ex2Lat;
                pathLng[3] = ex3Lng;  //ex6
                pathLat[4] = ex3Lat;
                pathLng[4] = ex3Lng;  //ex9
                pathLat[5] = ex2Lat;
                pathLng[5] = ex2Lng;  //ex5
                pathLat[6] = exLat;
                pathLng[6] = exLng;   //ex1
                pathLat[7] = exLat;
                pathLng[7] = exLng;   //ex1
                pathLat[8] = exLat;
                pathLng[8] = exLng;   //ex1
                break;

            case 5:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = ex2Lat - addLat;
                pathLng[1] = exLng;   //ex4
                pathLat[2] = ex3Lat - addLat * 2.0;
                pathLng[2] = exLng;   //ex7
                pathLat[3] = ex3Lat - addLat * 2.0;
                pathLng[3] = ex2Lng + addLng;  //ex8
                pathLat[4] = ex3Lat - addLat * 2.0;
                pathLng[4] = ex3Lng + addLng * 2.0;  //ex9
                pathLat[5] = ex2Lat - addLat;
                pathLng[5] = ex3Lng + addLng * 2.0;  //ex6
                pathLat[6] = exLat;
                pathLng[6] = ex3Lng + addLng * 2.0;  //ex3
                pathLat[7] = exLat;
                pathLng[7] = ex2Lng + addLng;  //ex2
                pathLat[8] = exLat;
                pathLng[8] = exLng;   //ex1
                break;

            case 6:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = ex2Lat - addLat;
                pathLng[1] = exLng;   //ex4
                pathLat[2] = ex2Lat - addLat;
                pathLng[2] = ex2Lng + addLng;  //ex5
                pathLat[3] = ex3Lat - addLat * 2.0;
                pathLng[3] = ex2Lng + addLng;  //ex8
                pathLat[4] = ex3Lat - addLat * 2.0;
                pathLng[4] = ex3Lng + addLng * 2.0;  //ex9
                pathLat[5] = ex2Lat - addLat;
                pathLng[5] = ex3Lng + addLng * 2.0;  //ex6
                pathLat[6] = exLat;
                pathLng[6] = ex3Lng + addLng * 2.0;  //ex3
                pathLat[7] = exLat;
                pathLng[7] = ex2Lng + addLng;  //ex2
                pathLat[8] = exLat;
                pathLng[8] = exLng;   //ex1
                break;

            case 7:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = exLat;
                pathLng[1] = ex2Lng + addLng;  //ex2
                pathLat[2] = exLat;
                pathLng[2] = ex3Lng + addLng * 2.0;  //ex3
                pathLat[3] = ex2Lat - addLat;
                pathLng[3] = ex3Lng + addLng * 2.0;  //ex6
                pathLat[4] = ex3Lat - addLat * 2.0;
                pathLng[4] = ex3Lng + addLng * 2.0;  //ex9
                pathLat[5] = ex3Lat - addLat * 2.0;
                pathLng[5] = ex2Lng + addLng;  //ex8
                pathLat[6] = ex3Lat - addLat * 2.0;
                pathLng[6] = exLng;   //ex7
                pathLat[7] = ex2Lat - addLat;
                pathLng[7] = exLng;   //ex4
                pathLat[8] = exLat;
                pathLng[8] = exLng;   //ex1
                break;

            case 8:
                pathLat[0] = exLat;
                pathLng[0] = exLng;   //ex1
                pathLat[1] = exLat;
                pathLng[1] = ex2Lng + addLng;  //ex2
                pathLat[2] = exLat;
                pathLng[2] = ex3Lng + addLng * 2.0;  //ex3
                pathLat[3] = ex2Lat - addLat;
                pathLng[3] = ex3Lng + addLng * 2.0;  //ex6
                pathLat[4] = ex3Lat - addLat * 2.0;
                pathLng[4] = ex3Lng + addLng * 2.0;  //ex9
                pathLat[5] = ex2Lat - addLat;
                pathLng[5] = ex2Lng + addLng;  //ex5
                pathLat[6] = exLat;
                pathLng[6] = exLng;   //ex1
                pathLat[7] = exLat;
                pathLng[7] = exLng;   //ex1
                pathLat[8] = exLat;
                pathLng[8] = exLng;   //ex1
                break;*/
        }

        //軌跡描画
        ArrayList<LatLng> current_points = null;
        PolylineOptions current_lineOptions = null;
        for (int i = 0; i < 1; i++) {
            current_points = new ArrayList<LatLng>();
            current_lineOptions = new PolylineOptions();
            for (int drow_val = 0; drow_val <= 17; drow_val++) {
                double drowLat = pathLat[drow_val];
                double drowLng = pathLng[drow_val];
                current_points.add(new LatLng(drowLat, drowLng));
            }
            //ポリライン
            current_lineOptions.addAll(current_points);
            current_lineOptions.width(5);
            current_lineOptions.color(Color.BLUE);
        }
        mMap.addPolyline(current_lineOptions);

        ex1 = new LatLng(exLat, exLng);
        ex2 = new LatLng(ex2Lat, exLng);
        ex3 = new LatLng(ex3Lat, ex2Lng);
        ex4 = new LatLng(ex3Lat, ex3Lng);
        ex5 = new LatLng(ex2Lat, ex4Lng);
        ex6 = new LatLng(ex4Lat, ex4Lng);
        ex7 = new LatLng(ex5Lat, ex3Lng);
        ex8 = new LatLng(ex5Lat, ex2Lng);
        ex9 = new LatLng(ex4Lat, exLng);
        ex10 = new LatLng(ex2Lat, exLng);
        ex11 = new LatLng(ex3Lat, ex5Lng);
        ex12 = new LatLng(ex3Lat, ex6Lng);
        ex13 = new LatLng(ex2Lat, ex7Lng);
        ex14 = new LatLng(ex4Lat, ex7Lng);
        ex15 = new LatLng(ex5Lat, ex6Lng);
        ex16 = new LatLng(ex5Lat, ex5Lng);
        ex17 = new LatLng(ex4Lat, exLng);
    }

    //スタートボタンを押した際誘導開始　　タイマーによりここがループ
    public class MainTimerTask extends TimerTask {
        public void run() {
            timerHandler.post(new Runnable() {
                public void run() {
                    if (connectFlg && startCount == 1) {
                        try {
                            MDeg = mmInStream.read() * 2;//6軸センサによる方位測定
                            if (MDeg > 180) {
                                MDeg = MDeg - 360;
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }

                    SDeg = (int) sensorChangeEvent.Deg;

                    //最終座標に着いたら終了
                    if (path_val == 17) {
                        //次の座標までの距離計算　results[0]…2点間の距離
                        Location.distanceBetween(currentLat, currentLng, pathLat[path_val], pathLng[path_val], results);
                        //次の座標までの角度計算　results[1]…2点間の角度
                        if(mode == 1)target_deg = (int) results[1] - (int) sensorChangeEvent.Deg;  //(Googlemap2点間の角度)　-　(地磁気センサ)
                        if(mode == 2)target_deg = (int) results[1] - MDeg; //6軸センサによる方位算出
                        if (target_deg > 180) target_deg = target_deg - 360;
                        else if (target_deg < -180) target_deg = target_deg + 360;

                        //テキスト表示
                        targetPoint.setText("targetLat:" + String.format("%, 6f", pathLat[path_val])
                                + "\n" + "targetLng:" + String.format("%, 6f", pathLng[path_val]));
                        status.setText(/*"次の座標まで" + results[0] + "[m]   角度" + target_deg + "[deg]"
                                + */ "盲導盤角度" + MDeg + "°　　スマホ角度" + sensorChangeEvent.Deg + "°　　初期角度" + DegOffSet);

                        if (results[0] < 2.0) {
                            Toast toast = Toast.makeText(getApplicationContext(), "FINISH!!", Toast.LENGTH_SHORT);
                            toast.show();
                            //盲導盤に停止の合図
                            if (connectFlg) {
                                try {
                                    mmOutputStream.write("0".getBytes());  //123456以外の数字送ると止まる
                                    startCount = 0;
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                            //計測終了
                            if (null != mainTimer) {
                                mainTimer.cancel();
                                mainTimer = null;
                                mainTimerTask = null;
                            }
                        } else {
                            if (connectFlg) outputToDevice(target_deg);
                        }
                    }

                    else if (path_val == 0) {
                        Location.distanceBetween(currentLat, currentLng, startLat, startLng, results1);
                        Location.distanceBetween(currentLat, currentLng, pathLat[path_val], pathLng[path_val], results2);
                        Location.distanceBetween(startLat, startLng, pathLat[path_val], pathLng[path_val], results3);
                        Location.distanceBetween(startLat, startLng, currentLat, currentLng, results4);
                        waypointDeg = Math.abs((int) results3[1] - (int) results4[1]);
                        if (waypointDeg > 180) waypointDeg = 360 - waypointDeg;
                        objectDistance = results1[0] * Math.sin(Math.toRadians(waypointDeg));//被験者から経路までの距離
                        closestDistance = results1[0] * Math.cos(Math.toRadians(waypointDeg));//前のWaypointから最近点までの距離
                        //前のWaypointから誘導点までの距離
                        if (objectDistance <= maxDistance) {
                            guideDistance = closestDistance + (maxDistance - objectDistance);
                        } else
                            guideDistance = closestDistance;
                        if (guideDistance > results3[0]) {
                            guideDistance = results3[0];

                        }
                        guidePoint = computeOffset(new LatLng(startLat, startLng), guideDistance, results3[1]);//誘導点の座標

                        Location.distanceBetween(currentLat, currentLng, guidePoint.latitude, guidePoint.longitude, results);
                        if(mode == 1)target_deg = (int) results[1] - (int) sensorChangeEvent.Deg;  //(Googlemap2点間の角度)　-　(地磁気センサ)
                        if(mode == 2)target_deg = (int) results[1] - MDeg; //6軸センサによる方位算出

                        if (target_deg > 180) {
                            target_deg = target_deg - 360;
                        } else if (target_deg < -180) {
                            target_deg = target_deg + 360;
                        }

                        LatLng current_pos = new LatLng(currentLat, currentLng);
                        ArrayList<LatLng> current_points1 = new ArrayList<>();
                        current_points1.clear();
                        current_points1.add(current_pos);
                        current_points1.add(guidePoint);
                        PolylineOptions straight = new PolylineOptions().addAll(current_points1)
                                .geodesic(false)  // 直線
                                .color(Color.GREEN)
                                .width(3);
                        mMap.addPolyline(straight);


                        LatLng closestPoint = computeOffset(new LatLng(startLat, startLng), closestDistance, results3[1]);//最近点の座標
                        Location.distanceBetween(pathLat[path_val], pathLng[path_val], closestPoint.latitude, closestPoint.longitude, results5);//最近点からWaypointまでの距離

                        //最近点と目標マーカーとの距離が0.75[m]以下になったら目標を次のマーカーへ切り替える
                        if (results5[0] < 0.75) {
                            LatLng position = new LatLng(pathLat[path_val], pathLng[path_val]);
                            options.position(position);          //更新したマーカーは緑に
                            BitmapDescriptor icon = BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN);
                            options.icon(icon);
                            mMap.addMarker(options);
                            path_val++;  //次のマーカーの更新
                        } else {
                            if (connectFlg) outputToDevice(target_deg);
                        }
                        targetPoint.setText("targetLat:" + String.format("%, 6f", pathLat[path_val])
                                + "\n" + "targetLng:" + String.format("%, 6f", pathLng[path_val]));
                        status.setText(/*"次の座標まで" + results[0] + "[m]   角度" + target_deg + "[deg]"
                                + */ "盲導盤角度" + MDeg + "°　　スマホ角度" + sensorChangeEvent.Deg + "°　　初期角度" + DegOffSet);
                    }

                    else {
                        Location.distanceBetween(currentLat, currentLng, pathLat[path_val - 1], pathLng[path_val - 1], results1);
                        Location.distanceBetween(currentLat, currentLng, pathLat[path_val], pathLng[path_val], results2);
                        Location.distanceBetween(pathLat[path_val - 1], pathLng[path_val - 1], pathLat[path_val], pathLng[path_val], results3);
                        Location.distanceBetween(pathLat[path_val - 1], pathLng[path_val - 1], currentLat, currentLng, results4);
                        waypointDeg = Math.abs((int) results3[1] - (int) results4[1]);
                        if (waypointDeg > 180) waypointDeg = 360 - waypointDeg;
                        objectDistance = results1[0] * Math.sin(Math.toRadians(waypointDeg));//被験者から経路までの距離
                        closestDistance = results1[0] * Math.cos(Math.toRadians(waypointDeg));//前のWaypointから最近点までの距離
                        //前のWaypointから誘導点までの距離
                        if (objectDistance <= maxDistance) {
                            guideDistance = closestDistance + (maxDistance - objectDistance);
                        } else
                            guideDistance = closestDistance;
                        if (guideDistance > results3[0]) {
                            guideDistance = results3[0];

                        }
                        guidePoint = computeOffset(new LatLng(pathLat[path_val - 1], pathLng[path_val - 1]), guideDistance, results3[1]);//誘導点の座標

                        Location.distanceBetween(currentLat, currentLng, guidePoint.latitude, guidePoint.longitude, results);
                        if(mode == 1)target_deg = (int) results[1] - (int) sensorChangeEvent.Deg;  //(Googlemap2点間の角度)　-　(地磁気センサ)
                        if(mode == 2)target_deg = (int) results[1] - MDeg; //6軸センサによる方位算出

                        if (target_deg > 180) {
                            target_deg = target_deg - 360;
                        } else if (target_deg < -180) {
                            target_deg = target_deg + 360;
                        }

                        LatLng current_pos = new LatLng(currentLat, currentLng);
                        ArrayList<LatLng> current_points1 = new ArrayList<>();
                        current_points1.clear();
                        current_points1.add(current_pos);
                        current_points1.add(guidePoint);
                        PolylineOptions straight = new PolylineOptions().addAll(current_points1)
                                .geodesic(false)  // 直線
                                .color(Color.GREEN)
                                .width(3);
                        mMap.addPolyline(straight);


                        LatLng closestPoint = computeOffset(new LatLng(pathLat[path_val - 1], pathLng[path_val - 1]), closestDistance, results3[1]);//最近点の座標
                        Location.distanceBetween(pathLat[path_val], pathLng[path_val], closestPoint.latitude, closestPoint.longitude, results5);//最近点からWaypointまでの距離

                        //最近点と目標マーカーとの距離が0.75[m]以下になったら目標を次のマーカーへ切り替える
                        if (results5[0] < 0.75) {
                            LatLng position = new LatLng(pathLat[path_val], pathLng[path_val]);
                            options.position(position);          //更新したマーカーは緑に
                            BitmapDescriptor icon = BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN);
                            options.icon(icon);
                            mMap.addMarker(options);
                            path_val++;  //次のマーカーの更新
                        } else {
                            if (connectFlg) outputToDevice(target_deg);
                        }
                        targetPoint.setText("targetLat:" + String.format("%, 6f", pathLat[path_val])
                                + "\n" + "targetLng:" + String.format("%, 6f", pathLng[path_val]));
                        status.setText(/*"次の座標まで" + results[0] + "[m]   角度" + target_deg + "[deg]"
                                + */ "盲導盤角度" + MDeg + "°　　スマホ角度" + sensorChangeEvent.Deg + "°　　初期角度" + DegOffSet);
                    }



                    if(startCount == 1) {
                        //保存用
                        array1[measure_val] = currentLat;
                        array2[measure_val] = currentLng;
                        array3[measure_val] = MDeg;
                        array4[measure_val] = SDeg;
                        array5[measure_val] = target_deg;
                        array6[measure_val] = mode;
                        array7[measure_val] = guidePoint.longitude;
                        array8[measure_val] = guidePoint.latitude;
                        measure_val++;
                    }

                    //軌跡描画
                    ArrayList<LatLng> current_points = null;
                    PolylineOptions current_lineOptions = null;
                        for(
                    int i = 0;
                    i< 1;i++)

                    {
                        current_points = new ArrayList<LatLng>();
                        current_lineOptions = new PolylineOptions();
                        for (int drow_val = 0; drow_val < measure_val; drow_val++) {
                            double drowLat = array1[drow_val];
                            double drowLng = array2[drow_val];
                            current_points.add(new LatLng(drowLat, drowLng));
                        }
                        //ポリライン
                        current_lineOptions.addAll(current_points);
                        current_lineOptions.width(5);
                        current_lineOptions.color(Color.RED);
                    }
                        mMap.addPolyline(current_lineOptions);
                }
            });
        }
    }

    public void outputToDevice(int deg) {
        String direction = null;
        if (deg < -67.5) direction = "1";  //左旋回
        else if (-67.5 <= deg && deg < -22.5) direction = "2";  //左前
        else if (-22.5 <= deg && deg <= 22.5) direction = "3";  //前　
        else if (22.5 < deg && deg <= 67.5) direction = "4";  //右前
        else if (67.5 < deg) direction = "5";  //右旋回
        if (/*!direction.equals(output)&& */startCount == 1) {
            output = direction;
            try {
                mSocket.connect();
                mmInStream = mSocket.getInputStream();
                mmOutputStream = mSocket.getOutputStream();
                mmOutputStream.write(output.getBytes()); //arduino側はchar v で受け取る
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if(startCount == 0){
            output = "0";
            try {
                mSocket.connect();
                mmInStream = mSocket.getInputStream();
                mmOutputStream = mSocket.getOutputStream();
                mmOutputStream.write(output.getBytes()); //arduino側はchar v で受け取る
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


    //GPSのプロバイダ関連のイベント
    //@Override ここの対処必要　LocationListenerのせいで消せない　使わないと思うけど
    public void onProviderDisabled(String provider) {
        gpsState.setText(R.string.disabled_provider);
    }

    public void onProviderEnabled(String provider) {
        gpsState.setText(R.string.enabled_provider);
    }


    public void onStatusChanged(String provider, int status, Bundle extras) {
        switch (status) {
            case LocationProvider.AVAILABLE:
                gpsState.setText(R.string.available);
                break;
            case LocationProvider.OUT_OF_SERVICE:
                gpsState.setText(R.string.out_of_service);
                break;
            case LocationProvider.TEMPORARILY_UNAVAILABLE:
                gpsState.setText(R.string.temp_unavailable);
                break;
        }
    }
}
