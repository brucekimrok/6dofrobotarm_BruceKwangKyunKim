/* 

로봇팔 프로그래밍 응용_by 파이어웍스_2019.12.30

1. RB038-1구매 6자유도 상용 로봇 코딩(PCA9685 서보드라이버, PS2X 플스조정기)을 참고함. 

2. 좌표 줄 때 이동하는 코딩을 적용함. 
   (Inverse kinematics 함수를 PCA9685사용토록 변영)
   ----------------------------------
   This sketch was shamelessly lifted intact from
   www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
   in an article "Robotic Arm Inverse Kinematics on Arduino By Oleg Mazurov".

   It was written to be used with a Lynxmotion AL5D robotic arm and a Renbotics
   servo shield.  The Renbotics shield uses a PCA9685 IC described below by NXP.
   
   This could be modified to use the Arduino Mega 2560 board which has 15 PWM pins.
   ----------------------------------
   
3. 해당 좌표를 Pixy2에서 읽어 오도록 적용함.
   (???)

*/


