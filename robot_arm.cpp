#include "Arduino.h"
#include "robot_arm.h" 
#include <Wire.h>    
#include "PCA9685.h"  //""는 유저 헤더파일, 위치가 .ino와 같은 곳.

PCA9685 pwmController;  //서보드라이버 객체생성

PCA9685_ServoEvaluator pwmServo1;  //서보모터 객체생성.
PCA9685_ServoEvaluator pwmServo2;  
PCA9685_ServoEvaluator pwmServo3;
PCA9685_ServoEvaluator pwmServo4;
PCA9685_ServoEvaluator pwmServo5;
PCA9685_ServoEvaluator pwmServo6;

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

void setup() {    //초기 셋업
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  pwmController.resetDevices();  //서보드라이버를 reset. 
  pwmController.init(B000000);
  pwmController.setPWMFrequency(50);
  pwmController.setChannelPWM(6, pwmServo1.pwmForAngle(30));  //밑바닥 20도 편차
  pwmController.setChannelPWM(0, pwmServo2.pwmForAngle(-15));  //어깨 -15도 편차. (+)각도가 반시계방향 회전, (-)각도가 시계방향 회전
  pwmController.setChannelPWM(1, pwmServo3.pwmForAngle(22));  //팔꿈치 22도 편차 (+)각도가 반시계방향 회전
  pwmController.setChannelPWM(2, pwmServo4.pwmForAngle(20));  //손목 20도 편차 (+)각도가 시계방향
  pwmController.setChannelPWM(4, pwmServo5.pwmForAngle(0));  //손회전 0도 편차
  pwmController.setChannelPWM(5, pwmServo6.pwmForAngle(30)); //집게 열림각 30, 닫힘각 -5
  
  delay(3000);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
}

void loop()
{
 //zero_x();
 //delay(3000);
 //line();
 //circle();

  delay(5000); 
 
  set_arm(0, 330, 330, 30);
  delay(100); 
 
  

}


/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
//void set_arm( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void set_arm( float x, float y, float z, float grip_angle_d )
{
 float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
 /* Base angle and radial distance from x,y coordinates */ 
 float bas_angle_r = atan2( x, y );
 float bas_angle_d = degrees(bas_angle_r); // 디그리로 전환
 float rdist = sqrt(( x * x ) + ( y * y ));
 /* rdist is y coordinate for the arm */
 y = rdist;
 /* Grip offsets calculated based on grip angle */
 float grip_off_z = ( sin( grip_angle_r )) * GRIPPER; //기울어진 그리퍼의 높이방향 길이
 float grip_off_y = ( cos( grip_angle_r )) * GRIPPER; //기울어진 그리퍼의 수평거리
 /* Wrist position */
 float wrist_z = ( z - grip_off_z ) - BASE_HGT;   // 손목의 높이 (베이스로 부터) 
 float wrist_y = y - grip_off_y;                  // 손목의 수평거리  
 /* Shoulder to wrist distance ( AKA sw ) */
 float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );  //어깨부터 손목까지 길이 제곱
 float s_w_sqrt = sqrt( s_w );  //어깨~손목 길이
 /* s_w angle to ground */
 //float a1 = atan2( wrist_y, wrist_z );
 float a1 = atan2( wrist_z, wrist_y );  // 어깨~손목 각도 
 /* s_w angle to humerus */
 float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));  //어깨~손목 과 상완간 각도
 /* shoulder angle 어깨 각도 */
 float shl_angle_r = a1 + a2;
 float shl_angle_d = degrees( shl_angle_r );
 /* elbow angle 팔꿈치 각도 */
 float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
 float elb_angle_d = degrees( elb_angle_r );
 float elb_angle_dn = -( 180.0 - elb_angle_d );
 /* wrist angle 손목 각도 */
 float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
/* wrist rotation 손목 회전 */
float wro_angle_d = 90 ;  //추후 입력 방안 검토 요망. 

//그립 각도 범위 제한
grip_angle_d = constrain(grip_angle_d, -40, 40);

//RB038 PCA9685 객체 사용
pwmController.setChannelPWM(6, pwmServo1.pwmForAngle(bas_angle_d));  //밑바닥
pwmController.setChannelPWM(0, pwmServo2.pwmForAngle(shl_angle_d - 105));  //어깨
pwmController.setChannelPWM(1, pwmServo3.pwmForAngle(elb_angle_d + 22));  //팔꿈치
pwmController.setChannelPWM(2, pwmServo4.pwmForAngle(wri_angle_d + 20));  //손목
pwmController.setChannelPWM(4, pwmServo5.pwmForAngle(wro_angle_d));  //손회전
pwmController.setChannelPWM(5, pwmServo6.pwmForAngle(grip_angle_d)); //집게

//로봇팔 똑바로 세운 각도 - 테스트 결과
// pwmController.setChannelPWM(6, pwmServo1.pwmForAngle(20));  //밑바닥 20도 편차
// pwmController.setChannelPWM(0, pwmServo2.pwmForAngle(-20));  //어깨 -20도 편차. 시계방향이 (-)
// pwmController.setChannelPWM(1, pwmServo3.pwmForAngle(20));  //팔꿈치 20도 편차
// pwmController.setChannelPWM(2, pwmServo4.pwmForAngle(20));  //손목 20도 편차
// pwmController.setChannelPWM(4, pwmServo5.pwmForAngle(0));  //손회전 0도 편차
// pwmController.setChannelPWM(5, pwmServo6.pwmForAngle(30)); //집게 열림각 30, 닫힘각 -5

}

/* moves arm in y axis  ->  y축 이동 */
void zero_x()
{
 for( double yaxis = 200.0; yaxis < 250.0; yaxis += 1 ) {
   set_arm( 0, yaxis, 300.0, 0 );
   delay( 10 );
 }
 for( double yaxis = 250.0; yaxis > 200.0; yaxis -= 1 ) {
   set_arm( 0, yaxis, 300.0, 0 );
   delay( 10 );
 }
}

/* moves arm in a straight line  -> x 축 이동*/
void line()
{
   for( double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5 ) {
     set_arm( xaxis, 200, 100, 0 );
     delay( 10 );
   }
   for( float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5 ) {
     set_arm( xaxis, 200, 100, 0 );
     delay( 10 );
   }
}

/* y-z면위에 원 그리기 */
void circle()
{
 #define RADIUS 50.0
 float zaxis,yaxis;
 for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
     yaxis = RADIUS * sin( radians( angle )) + 200;
     zaxis = RADIUS * cos( radians( angle )) + 100; //높이 조절 요망.. 
     set_arm( 0, yaxis, zaxis, 0 );
     delay( 10 );
 }
}
