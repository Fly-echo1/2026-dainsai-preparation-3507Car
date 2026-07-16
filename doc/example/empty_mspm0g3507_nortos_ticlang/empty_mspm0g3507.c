/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 #include "ti/driverlib/m0p/dl_core.h"
 #include "ti_msp_dl_config.h"
 #include "board.h"
 #include "stdio.h"
 #include "bsp_motor.h"
 #include "bsp_track.h"
 #include "bsp_pid.h"
 #include "bsp_beep.h"
 #include "key.h"
 #include "bsp_gyro.h"
 #include "stdlib.h"
 #include "oled.h"
 
 int tc=0;//推出标志
  
 int q;//圈数，可以设置成局部变量，但是我为了方便，全都是全局变量，如果追求效率可以自行更改

 int a=1; //阶段标志（其实可以大概可以不需要，但是加上更加保险，心安）

 int start=1;//启动标志位，用于启动缓冲

 int32_t mile1 = 0, mile2 = 0,mile = 0;//编码器里程计数

 int shengguang_time=0;  //用于声光提示（我们的蜂鸣器和led灯是接在一起的，节省一个io口，方便）
 static int shengguang_flag=0;

 static int position_flag = 0; //位置环标志位，里程达到目标为1，否则为0

 int32_t Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt,PWMA,PWMB; //编码器数据

 int32_t motor_speeda = 800,motor_speedb = 800;//电机速度控制，0为停止
 int32_t motor_mode = MOTOR_STOP;//0默认停止  1速度环  2位置环  3循迹
//  char str[10];
//  float ang;

 void beep(void)
 {
     if(shengguang_flag == 0)
     {
            beep_on();      
         shengguang_flag = 1;
     }
 }
 int main(void)
 {  
     beep_init();
     board_init(); 
     gyro_init();
     motor_init(); //开启电机编码器引脚外部中断
     track_init(); //开启循迹模块初始化，该函数没有作用，只是提醒工程师有配置循迹模块
     NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
     DL_TimerA_startCounter(TIMER_0_INST);
     OLED_Init();
 
 //问题一
 while (key_scan()==1)
 {      
         if (key_scan()==tc)
         {
            motor_stop(); 
             break;
         }
        gyro_init();
        if(start==1)
         {delay_ms(500);start=2;};		

    //   while (1) {
    //     ang=get_angle();
    //     sprintf(str,"yaw=%.3f",ang);
    //   motor_mode = MOTOR_SPEED;
    //   motor_speeda = 800 - gyro_pid(0);
    //   motor_speedb = 800 + gyro_pid(0);
    //   OLED_ShowNum(0, 0, encoderA_cnt, 8, 8, 1);
    //   OLED_ShowNum(0, 10, encoderB_cnt, 8, 8, 1);
    //   OLED_ShowNum(0, 20, PWMA, 8, 8, 1);
    //   OLED_ShowNum(0, 30, PWMB, 8, 8, 1);
    //   OLED_ShowString(0, 40, str, 8, 1);
    //   OLED_Refresh();
    //   delay_ms(100);
    //   }  
       track_control();
       delay_ms(100);
        while (track_scan())
            {
                motor_mode = MOTOR_SPEED;
                motor_speeda = 800 - gyro_pid(0);
                motor_speedb = 800 + gyro_pid(0);
           }

                if(!track_scan())
                {
                    motor_mode = MOTOR_STOP;
                    beep();                                           
                    mile1 = 0;
                    mile2 = 0;
                    mile = 0;	
                    tc=1;
                    start=1;				
                }                
 }
 
 //问题二
 while (key_scan()==2)
 {
      if (key_scan()==tc)
         {
            mile1 = 0;
            mile2 = 0;
            mile = 0;	
             motor_stop();
             break;
         }
         if(start==1)
         {delay_ms(500);start=2;}
         track_control();
         delay_ms(100);
          while (track_scan())
      {      
         motor_mode = MOTOR_SPEED;
         motor_speeda = 800 - gyro_pid(0);
         motor_speedb = 800 + gyro_pid(0);
      }
             
         if (!track_control())
         {
                  
             while (tc!=2) //到达B点，进入第二阶段
             {	
                 switch (a)
              {
                 case 1:	{		//到达B点，提示作用，只执行一次
                             motor_mode = MOTOR_STOP;
                             beep();																
                             a=2;		//使进入第二阶段
                         }
                 case 2:{		                                                    
                            //第二阶段黑圆弧
                          while(!track_control())
                           {
                            mile1 = 0;
                            mile2 = 0;
                            mile = 0;
                           }	                               						                             
                        //到达C点                      	   							
                          beep();
                          a=3;		//跳出第二阶段，进入第三阶段		                                                                                                				                                                     									
                         }
                 case 3:{
                              //第三阶段，白直线                           
                             while (track_scan())
                          {                         
                             motor_mode = MOTOR_SPEED;
                             motor_speeda = 800 - gyro_pid(180);
                             motor_speedb = 800 + gyro_pid(180);                   
                          }										
                          if (!track_control())
                           {
                              //到达D点	  
                              motor_mode = MOTOR_STOP;                          
                              beep();	                          	
                              a=4;	//跳出第三阶段，进入第四阶段         
                           }	                        					    	                 						                              						 
                         }
                 case 4:{
                           //第四阶段，黑圆弧                         
                            while (get_angle() > 0 || get_angle() < -90) //防止循迹开头不稳
                             {
                                 track_control();
                                 mile1 = 0;
                                 mile2 = 0;
                                 mile = 0;    
                             }			
                            while(!track_control()) //接续循迹，不用角度控制更加简洁
                            {	
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;
                            }                          
                            //回到A点                       
                              beep();
                              tc=2;
                              a=1;
                              start=1;								                              											                                                     				 						
                        }
              }   
             }		
         }
 }
 
 //问题三
 while (key_scan()==3)
 {
           if (key_scan()==tc)
         {
             mile=0;	
             motor_stop();
             break;
         }
            if(start==1)
         {delay_ms(1000);start=2;};				             
             mile1 = 0;
             mile2 = 0;
             mile = 0;	            
             motor_mode = MOTOR_SPEED;  
              track_control();
             delay_ms(200);   
             while(track_scan())
              {                     
                motor_speeda = 800 - gyro_pid(50);
                motor_speedb = 800 + gyro_pid(50);
                 if (position_flag == 1)
                   {
                    motor_mode = MOTOR_STOP;   
                     while (track_control())
                      {
                    
                    Set_PWM(1500, 3000);
                      }   
                  }
              }            
    
         if (!track_control())
         {            
             while (tc!=3) //到达B点，进入第二阶段
             {	
                 switch (a)
              {
                 case 1:{		
                        //到达B点，提示作用，只执行一次
                            motor_mode = MOTOR_STOP;
                             beep();																
                             a=2;		//使进入第二阶段
                        }
                 case 2:{		
                           //第二段黑圆弧                  
                          while (get_angle() > -90 ) 
                          {
                             track_control();
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;  
                          }                             
                          while(!track_control())
                            {
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;
                            } 
                                //到达C点                                 		  									
                                beep();					
                                 a=3;		//跳出第二阶段，进入第三阶段		                                                                                                      				                                                     									
                         }
                 case 3:{
                              //第三阶段，白直线                        
                             motor_mode = MOTOR_SPEED;
                              while (track_scan())
                             {                           
                                  motor_speeda = 800 - gyro_pid(135);
                                  motor_speedb = 800 + gyro_pid(135);
                                if (position_flag == 2)
                                {
                                motor_mode = MOTOR_STOP;     
                                  while (track_control())
                                 {                                 
                                    Set_PWM(3200, 1300);
                                 }                                                                                      
                               }
                             }                                                                                                                     
                          if (!track_control())
                            {
                              //到达D点	                            
                              motor_mode = MOTOR_STOP;                            
                              beep();	                             		                          
                              a=4;	//跳出第三阶段，进入第四阶段         
                            }					    	                 						                              						 
                         }
                 case 4:{                                                       
                           while (get_angle() > 0 || get_angle() < -90) //防止循迹开头位置不稳
                            {
                             track_control();
                             mile1 = 0;
                             mile2 = 0;
                             mile = 0;
                            } 
                             while(!track_control())    //接续循迹，不用角度控制更加简洁
                            {
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;
                            }                          						
                             if (track_scan())
                              {
                                 //回到A点                       
                                 beep();
                                 tc=3; 
                                 a=1;
                                 start=1;
                             }
                             break;							                                                     				 						
                        }
              }   
             }		
         }  
 }
 
 //问题四
 while (key_scan() == 4)
 {
    if (key_scan() == tc)
         {
            mile=0;	
            motor_stop();
             break;
         }
         gyro_init();
          if(start==1)
         {  delay_ms(1000);start=2; }		
        track_control();
        delay_ms(100);
         
 for(q=0; q<4; q++)
     {                        
            a=1;     		
            mile1 = 0;
            mile2 = 0;
            mile = 0;
            position_flag = 0;
            motor_mode = MOTOR_SPEED;  
           while (track_scan())
              {                            
                  motor_speeda = 800 - gyro_pid(50);
                  motor_speedb = 800 + gyro_pid(50);
                  if (position_flag == 1)
                   {
                     motor_mode = MOTOR_STOP;   
                     while (track_control())
                      {
                    
                    Set_PWM(1500, 3000);
                      }   
                  }
              }		
           
         if (!track_scan())
         {             	
              //到达B点，进入第二阶段				
                 switch (a)
              {
                 case 1:{		
                         //到达B点，提示作用	
                           motor_mode = MOTOR_STOP;						
                             beep();	                          					                         																											
                             a=2;		//使进入第二阶段
                         }
                 case 2:{		
                             //第二段黑圆弧                  
                          while (get_angle() > -90 )  //防止循迹开头位置不稳
                            {
                             track_control();
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;  
                            }                             
                          while(!track_control()) //接续循迹，不用角度控制更简洁
                            {
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;
                            } 
                                //到达C点                                 		  									
                                beep();					
                                 a=3;		//跳出第二阶段，进入第三阶段		   	                                							                                              				                                                     									
                        }
                 case 3:{
                              motor_mode = MOTOR_SPEED;
                              position_flag = 0;
                              while (track_scan())//第三段白直线
                             {                                                         
                                  motor_speeda = 800 - gyro_pid(135);
                                  motor_speedb = 800 + gyro_pid(135);                            
                               if (position_flag == 2)
                                {
                                  motor_mode = MOTOR_STOP;     
                                  while (track_control())
                                 {                                 
                                    Set_PWM(3200, 1300);
                                 }                                                                                    
                               }                             
                             }                                                                                                                 
                          if (!track_control())
                            {
                              //到达D点	                            
                               motor_mode = MOTOR_STOP;                            
                               beep();	                             		                          
                               a=4;	//跳出第三阶段，进入第四阶段         
                            }					    	                 	    	                 						                              						 
                        }
                 case 4:{                                                       
                           //第四段黑圆弧     																						                                 				                            	                                                                    
                           while (get_angle() > 0 || get_angle() < -90)  //防止循迹开头不稳
                           {
                              track_control();
                               mile1 = 0;
                               mile2 = 0;
                               mile = 0;
                           }                          
                            while(!track_control())  //接续循迹，没有角度控制更加简洁
                            {
                                mile1 = 0;
                                mile2 = 0;
                                mile = 0;
                            }  
                                //回到A点  
                            if (track_scan())
                            {
                            beep(); 
                            } 
                                                                                                  
                        }
              }   				
         }
       }
       a=1;
       start=1;
       tc=4;
 }
 }
     
 void TIMER_0_INST_IRQHandler(void)
 {          
     //编码器速度计算
     if( DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO )
     {
         encoderA_cnt = Get_Encoder_countA;//两个电机安装相反，所以编码器值也要相反
         encoderB_cnt = -Get_Encoder_countB;
         mile1 += Get_Encoder_countB;
         mile2 += Get_Encoder_countA;
         mile = (abs(mile1) + abs(mile2)) / 2;
         Get_Encoder_countA = 0;//编码器计数值清零
         Get_Encoder_countB = 0;
        //位置环
         if ((abs(mile) > 4080) && track_scan())
         {                     
            motor_mode = MOTOR_STOP;                             
            if ( get_angle() < 90)
            {                   
                 position_flag = 1;
                                                        				
            }
           else if ( get_angle() > 90) 
            {                                     
                 position_flag = 2;                         
            }              
        
         }
         //速度环
         if( motor_mode == MOTOR_SPEED )
         {
             PWMA = Velocity_A(motor_speeda, encoderA_cnt * 100);//指定两轮速度
             PWMB = Velocity_B(motor_speedb, encoderB_cnt * 100);
             Set_PWM(PWMA,PWMB);
         }
         //声光
          if(shengguang_flag == 1)
        {
            shengguang_time ++;
            if(shengguang_time >= 10)
            {
                beep_off();					
                shengguang_flag = 0;
                shengguang_time = 0;
            }
        }
     }
 }
 
    
 