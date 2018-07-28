package com.sensor;

import android.app.Activity;
import android.content.Context;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.Menu;
import android.widget.TextView;

public class Sensor extends Activity implements SensorEventListener {
	
	
	private GraphView mGraph_kalman;
	private GraphView mGraph_comp;
	private GraphView mGraph1;
	private SensorManager mSensorManager;
	private android.hardware.Sensor mAccelerometer;
	private android.hardware.Sensor mGyro;
	private TextView teta_txt_kalman;
	private TextView teta_txt_comp;
	private TextView teta_txt2;
	private TextView pwm_txt;
	private float timestamp=0;
	private float dt;
	private static final float NS2S = 1.0f / 1000000000.0f;
	private float angle;
	private float newtimestamp=0;
	private float Kp=0.9f;
	private float Ki=0.5f;
	private float Kd=0.5f;
	private float pwm;
	private float hata;
	float Q_angle  =  0.001f; //0.001
    float Q_gyro   =  0.003f;  //0.003
    float R_angle  =  0.1f;  //0.03

    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    float y, S;
    float K_0, K_1;
	
	float x_acc;
	float y_acc;
	float z_acc;
	
	float x_gyro;
	float y_gyro;
	float z_gyro;

	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.sensor_layout);
		
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		mAccelerometer = mSensorManager.getDefaultSensor(android.hardware.Sensor.TYPE_ACCELEROMETER);
		mGyro = mSensorManager.getDefaultSensor(android.hardware.Sensor.TYPE_GYROSCOPE);
		mGraph_kalman = (GraphView)findViewById(R.id.graph_kalman);
		mGraph_comp = (GraphView)findViewById(R.id.graph_comp);
		mGraph1 = (GraphView)findViewById(R.id.graph1);
		
		mGraph_kalman.setMaxValue(360);
		mGraph_kalman.setColor(1);
		mGraph_comp.setMaxValue(360);
		mGraph_comp.setColor(3);
		mGraph1.setMaxValue(360);
		mGraph1.setColor(0);
		
				
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.sensor_layout, menu);
		return true;
	}
	
	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
		mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_GAME);
		mSensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_GAME);
	}
	
	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
		mSensorManager.unregisterListener(this);
		timestamp=0.0f;
	}

	@Override
	public final void onAccuracyChanged(android.hardware.Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub
		
		
	}
	 
	@Override
	public final void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub
		
		
		
		if(event.sensor.getType() == android.hardware.Sensor.TYPE_ACCELEROMETER){
		x_acc = event.values[0];
		y_acc = event.values[1];
		z_acc = event.values[2];
		}
		
		if(event.sensor.getType() == android.hardware.Sensor.TYPE_GYROSCOPE){
		x_gyro = event.values[0];
		y_gyro = event.values[1];
		z_gyro = event.values[2];
		}
		
		if(timestamp != 0){
		
	    newtimestamp = System.nanoTime();
		dt = (newtimestamp - timestamp) * NS2S;
		
		if(dt != 0 ){


		    float newAngle = (float) Math.atan2(z_acc,y_acc);
		    
		    
		    x_angle +=  dt*(-x_gyro - x_bias);
		    P_00 +=  -  dt*(P_10 + P_01) + Q_angle;
		    P_01 +=  -  dt*P_11;
		    P_10 +=  -  dt*P_11;
		    P_11 +=  + Q_gyro*dt;

		    y = newAngle - x_angle;
		    S = P_00 + R_angle;
		    K_0 = P_00 / S;
		    K_1 = P_10 / S;

		    x_angle +=  K_0 * y;
		    x_bias  +=  K_1 * y;
		    P_00 -= K_0 * P_00;
		    P_01 -= K_0 * P_01;
		    P_10 -= K_1 * P_00;
		    P_11 -= K_1 * P_01;
		    
		    angle = 0.97f*(angle - x_gyro*dt) + 0.03f*newAngle;
		    
		    float angle_degree_comp = (float) Math.toDegrees(angle);

		    
		    float angle_degree_kalman = (float) Math.toDegrees(x_angle);

	    
	    hata=angle_degree_kalman-90;
	    pwm= (Kp*hata)-(Kd*x_gyro);
	    

		teta_txt_kalman = (TextView)findViewById(R.id.teta_kalman);
		teta_txt_comp = (TextView)findViewById(R.id.teta_complementary);
		teta_txt2 = (TextView)findViewById(R.id.teta2);
		pwm_txt = (TextView)findViewById(R.id.pwm);
		
		String teta_kalman = String.format("%.2f", angle_degree_kalman);
		
		String teta_comp = String.format("%.2f", angle_degree_comp);
		
		String teta2 = String.format("%.2f", Math.toDegrees(newAngle));
		
		String pwm_text = String.format("%.2f", pwm);
		
		mGraph_kalman.addDataPoint(angle_degree_kalman+180);
		mGraph_comp.addDataPoint(angle_degree_comp+180);
		mGraph1.addDataPoint((float) Math.toDegrees(newAngle)+180);
		
		
		teta_txt_kalman.setText(teta_kalman);
		teta_txt_comp.setText(teta_comp);
		teta_txt2.setText(teta2);
		pwm_txt.setText(pwm_text);
		
		}
		}
		timestamp = System.nanoTime();
		
	}
	
}
