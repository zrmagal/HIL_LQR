/*! \file 1.c
\brief Example: minimal code for ADC input.

This file contains an short and simple example for text output of the
analog input lines. It's designed for the description pages and shows
the basic usage of libpruio with a minimum of source code, translatable
between FreeBASIC and C.

Licence: GPLv3

Copyright 2014 by Thomas{ dOt ]Freiherr[ At ]gmx[ DoT }net


Compile by: `gcc -Wall -o 1 1.c -lpruio`

*/

////////////////////////////////////////////////////////////////////////////
//LQR Calculado para o Modelo do Satélite de Reurison Silva Rodrigue///////
//Utilizando a DAC6703 para leitura de estados e UDP para envio do comando//
////////////////////////////////////////////////////////////////////////////

#include <sys/stat.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <netdb.h>
#include <netinet/in.h>

//BIBLIOTECAS NECESSARIAS PARA ANALOG_READ
#include <time.h>
#include <sys/time.h>
#include "pruio.h"
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include "pruio_pins.h"

//BIBLIOTECAS NECESSARIAS PARA O UDP
#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);
#include <arpa/inet.h>
#include <sys/socket.h>
#include <math.h>

//DEFINES PARA O UDP
#define SERVER "10.10.10.6" //Endereço do TARGET Simulink Real Time.
#define BUFLEN 256  //Max length of bu_ffer, Antes : 512
#define PORT 65000  //The port on which to send data


//Limites superiores e inferiores calculados no MATLAB
#define IN_1_MAX 3*1000.0
#define IN_1_MIN -7*1000.0

#define IN_2_MAX 18*1000.0
#define IN_2_MIN 0*1000.0

#define IN_3_MAX 0.5*1000.0 //-3.1
#define IN_3_MIN -1.5*1000.0 //+0.0

#define IN_4_MAX 0.2*1000.0 //-1.0
#define IN_4_MIN -0.4*1000.0 //+0.05

#define IN_5_MAX 0.2*1000.0 //-0.03
#define IN_5_MIN -0.4*1000.0 //+0.70

#define IN_6_MAX 1.5*1000.0 //-0.05
#define IN_6_MIN -3*1000.0 //+1.17

//#define BUFLEN 256  //Max length of bu_ffer (Num. de estados x 8(bytes))
#define PORT 65000   //The port on which to listen for incoming data

#define KU (0.001027120000266)
#define L  (2.69)
#define MU_G (8.82) 

double mapRange(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void die(char *s)
{
    perror(s);
    exit(1);
}

int main(void)
{

    double state_v, state_u, state_yaw_rate, state_roll_rate, state_roll_angle, front_steer; //Adc Value GPIO [0, 65520]
    double state_v_f, state_u_f, state_yaw_rate_f, state_roll_rate_f, state_roll_angle_f, front_steer_f; //Estados do unidades V.
    int state_v_ADC, state_u_ADC, state_yaw_rate_ADC, state_roll_rate_ADC, state_roll_angle_ADC, front_steer_ADC; //Leitura do valor inteiro do ADC da Beaglebone.

    struct sockaddr_in si_me, si_other;
    int s, slen = sizeof(si_other), recv_len;
    double u[7] = {0}; //vetor de comando do lqr.

    //Ganhos calculados offline no MATLAB.
    double k11 = -8440.824774772678;
    double k12 =  6631.142825552018;
    double k13 =  -241.286235287233;
    double k14 = -1416.508821362820;
    double desired_yaw_rate;
    double max_desired_yaw_rate;
    double slip_angle; 
    double total_speed2;
   
    //------------------------------------------------------------------
    //-----------------  Criacao do socket UDP -------------------------
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);

    if (inet_aton(SERVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }

    printf("Socket client Criado\n");

    //------------------------------------------------------------------
    //------------------------------------------------------------------


    //------------------------------------------------------------------
    //-----------------Create new driver structure----------------------
    pruIo *io = pruio_new(PRUIO_DEF_ACTIVE, 0x98, 0, 1);

    if(io->Errr){
    	printf("initialisation failed (%s)\n\a\a",io->Errr);
    }

    // upload (default) settings, start IO mode
    if (pruio_config(io, 1, 0x1FE, 0, 4)){
	printf("Config failed (%s)\n",io->Errr);
    }

    //------------------------------------------------------------------
    //------------------------------------------------------------------

    else {

        while(1){

	    //Physical Pin : AIN0 (P9_39) - Estado 1.
	    state_v_ADC = io->Adc->Value[1];
            state_v = mapRange((double)state_v_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\n\n%d %f", state_v_ADC, state_v);
            state_v = mapRange(state_v, 0.0f, 1800.0f, IN_1_MIN, IN_1_MAX);
	    state_v_f = (double)state_v/1000.00;

	    //Physical Pin : AIN1 (P9_40) - Estado 2.
	    state_u_ADC = io->Adc->Value[2];
            state_u = mapRange((double)state_u_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\t%f",state_u);
	    state_u = mapRange(state_u, 0.0f, 1800.0f, IN_2_MIN, IN_2_MAX);
	    state_u_f = (double)state_u/1000.00;

            //Physical Pin : AIN2 (P9_37) - Estado 3.
            state_yaw_rate_ADC = io->Adc->Value[3];
            state_yaw_rate = mapRange((double)state_yaw_rate_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\t%f",state_yaw_rate);
	    state_yaw_rate = mapRange(state_yaw_rate, 0.0f, 1800.0f, IN_3_MIN, IN_3_MAX);
	    state_yaw_rate_f = (double)state_yaw_rate/1000.00;

	    //Physical Pin : AIN3 (P9_38) - Estado 4.
	    state_roll_rate_ADC = io->Adc->Value[4];
            state_roll_rate = mapRange((double)state_roll_rate_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\t%f",state_roll_rate);
	    state_roll_rate = mapRange(state_roll_rate, 0.0f, 1800.0f, IN_4_MIN, IN_4_MAX);
	    state_roll_rate_f = (double)state_roll_rate/1000.00;

	    //Physical Pin : AIN4 (P9_39) - Estado 5.
	    state_roll_angle_ADC = io->Adc->Value[5];
            state_roll_angle = mapRange((double)state_roll_angle_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\t%f",state_roll_angle);
	    state_roll_angle = mapRange(state_roll_angle, 0.0f, 1800.0f, IN_5_MIN, IN_5_MAX);
	    state_roll_angle_f = (double)state_roll_angle/1000.00;

	    //Physical Pin : AIN5 (P9_36) - Estado 6.
	    front_steer_ADC = io->Adc->Value[6];
            front_steer = mapRange((double)front_steer_ADC, 0.0f, 65520.0f, 0.0f, 1800.0f);
//	    printf("\n%f",front_steer);
	    front_steer = mapRange(front_steer, 0.0f, 1800.0f, IN_6_MIN, IN_6_MAX);
	    front_steer_f = (double)front_steer/1000.00;

  //  printf("\n valor %d :%f %f %f %f %f %f", sizeof(u), state_u_f,state_v_f,state_yaw_rate_f,state_roll_rate_f,state_roll_angle_f,front_steer_f);
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    	total_speed2 = state_u_f*state_u_f + state_v_f*state_v_f;
	desired_yaw_rate = fabs(front_steer_f*sqrt(total_speed2)/(L + L*KU*total_speed2));
	max_desired_yaw_rate = fabs(MU_G/sqrt(total_speed2)); 
	if( desired_yaw_rate > max_desired_yaw_rate )
	{
		desired_yaw_rate = max_desired_yaw_rate;
	}

	desired_yaw_rate = desired_yaw_rate*( front_steer_f > 0 ? 1.0 : -1.0) ;
	
	slip_angle = atan2(state_v_f,state_u_f);
	 
	

//	printf("\n erro  [e1] : %f , %f - [e2] : %f - [e3] : %f\n[e4] : %f \n", desired_yaw_rate, k12*(state_yaw_rate_f - desired_yaw_rate), k11*slip_angle, k13*state_roll_rate_f,k14*state_roll_angle_f);
	    
        //Cálculo do comando LQR.(Validado via udp)
	//Lei de controle do LQR (u = ud - K(x - xd))
	u[0] = k12*(desired_yaw_rate - state_yaw_rate_f) -
               k11*slip_angle -
	       k13*state_roll_rate_f -
               k14*state_roll_angle_f;

	// Coloca os estados no sinal que sera enviado
	u[1] = state_v_f;
	u[2] = state_u_f;
	u[3] = state_yaw_rate_f;
	u[4] = state_roll_rate_f;
	u[5] = state_roll_angle_f;
	u[6] = front_steer_f;	

	printf("\n %f",u[0]);
	//Envio do comando [u] para o Simulink Real Time
	if(sendto(s, &u, sizeof(u), 0, (struct sockaddr *) &si_other, slen) == -1)
	{
		die("sendto()");
	}

        } //fim do while.

    } //fim do else.

    close(s);
    pruio_destroy(io);
    return 0;

}//fim da main
