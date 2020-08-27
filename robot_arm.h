
/* Arm dimensions( mm ) */
#define BASE_HGT 65.00     //base height 
#define HUMERUS 105.00      //shoulder-to-elbow "bone" 
#define ULNA 150.00        //elbow-to-wrist "bone"
#define GRIPPER 180.00      //gripper (incl.heavy duty wrist rotate mechanism) length 

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

//서보모터드라이버 - PCA9685의 포트번호 
#define IN1 4  //손회전부분
#define IN2 5  //집게부분
#define IN3 6  //밑바닥부분
#define IN4 0  //어깨부분
#define IN5 1  //팔꿈치부분
#define IN6 2  //손목부분

//조종기 무선안테나 포트번호- 아두이노쉴드
#define PS2_DAT     12   
#define PS2_CMD     11  
#define PS2_SEL     10  
#define PS2_CLK     13  

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

void circle();
void line();
void zero_x();
void set_arm( float x, float y, float z, float grip_angle_d );



