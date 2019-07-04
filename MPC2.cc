/////////////INITIALIZATION///////////////////////////////////
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include "solver.h"
#include <chrono>
#include <fstream>
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <tuple>

using namespace std;
using namespace std::chrono;




msg msgMPC;//size should be 4000 times resample
pthread_mutex_t mtx;

Vars vars;
Params params;
Workspace work;
Settings settings;

DataMPC dataMPC;



#define NUM_THREADS 2



////////////////////////////////////////////////////////////////
void *ThreadsDistribution(void *threadid) {
   int id = (intptr_t) threadid;
   long tid;
   tid = (long)id;
   if (tid == 0){

		set_defaults();
		setup_indexing();



 		//int n = 3; //nr. of degrees of freedom (3 per joint)
 		int T = dataMPC.T;
 		double resample = dataMPC.resample;
 		double span = dataMPC.span;

		int internal_flag1 = 0;
		double record2[T+1];
 		int rtsize = ceil(span/resample);
 		double rt[rtsize];
		double tsim;
		double h;
		double hprev;
                double hmilis;
                double msgMPCsend[500];
		int a1;
		double b1;
		double i2;

                msg msgMPC2;

 		for(int k = 0; k<rtsize; k++){

 		    rt[k] = k*resample;
 		    if (rt[k] > span){
 		        rt[k] = span;
 		        }
 		    tsim = span - k*resample;
 		    h = tsim/T;
 		    model(h);

     		hprev = (span - (k-1)*resample)/T;


/////////////////////// HERE THE PROBLEM GETS PARTICULAR FOR EACH JOINT //////////////////
auto start = high_resolution_clock::now();
                for(int joint=1; joint<8.; joint++){
                    double posgoal = dataMPC.posgoal[joint-1];
                    goal(posgoal);
                      load_static_data(joint);

                      initial_state(hprev, resample, k, joint, dataMPC);

                      settings.verbose = 0;

                      solve();

                      store_next_initial(joint, vars);

                      record2[0] = *(params.z[0]+1);
                      hmilis = h*1000;


                            for(int j=0;j<T;j++){
                                    record2[j+1] = *(vars.z[j+1]+1);
                            }

                            i2 = 0;

                            for(int i1=0; i1<sizeof(msgMPC2.j7)/8.; i1++){
                                    a1 =floor(i2/hmilis);
                                    b1 = fmod(i2,hmilis);
                                    msgMPCsend[i1] = ((record2[a1+1]-record2[a1])/hmilis)*b1 + record2[a1];
                                    if(joint == 1){
                                        msgMPC2.j1[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 2){
                                        msgMPC2.j2[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 3){
                                        msgMPC2.j3[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 4){
                                        msgMPC2.j4[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 5){
                                        msgMPC2.j5[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 6){
                                        msgMPC2.j6[i1] = msgMPCsend[i1];
                                    }
                                    else if(joint == 7){
                                        msgMPC2.j7[i1] = msgMPCsend[i1];
                                    }
                                    i2++;
                            }
                }
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
               // cout <<duration.count()/1000 << " miliseconds" << endl;

  ////////////////// UNTIL HERE////////////////////////////////7
			internal_flag1 = 0;
                        internal_flag1 = send(internal_flag1, msgMPC2);
		}


   }
   else {
        //usleep(5000);
    	int internal_flag2 = 0;
	double time = 0.0;
        //int size_veloc7 = floor(dataMPC.span*2000);
        double veloc1[200];
        double veloc2[200];
        double veloc3[200];
        double veloc4[200];
        double veloc5[200];
        double veloc6[200];
        double veloc7[200]; //inizialite both two with int;
        double veloc7real[200];
	int indice;
	double resampleRobot = dataMPC.resample;
        double resampleRobot_ms = resampleRobot*1000;
	double timemax = dataMPC.span;

      franka::Robot robot("172.16.0.2");
      setDefaultBehavior(robot);
 // First move the robot to a suitable joint configuration
    array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
	usleep(50000);

	

        internal_flag2 = receive_first(internal_flag2, veloc1, veloc2, veloc3, veloc4, veloc5, veloc6, veloc7, msgMPC);

	
	robot.control(
        [=, &time, &internal_flag2, &veloc1, &veloc2, &veloc3, &veloc4, &veloc5, &veloc6, &veloc7, &timemax, &veloc7real, &indice](const franka::RobotState &robot_state, franka::Duration period) -> franka::JointVelocities{

      time += period.toSec();
			indice = floor(time*1000);
			if (fmod(floor(time*1000),resampleRobot_ms) == 0 && time != 0.0){
				internal_flag2 = 0;
                                internal_flag2 = receive(internal_flag2, veloc1, veloc2, veloc3, veloc4, veloc5, veloc6, veloc7, indice, msgMPC);
			}
	if(time>timemax){
                veloc1[indice] = 0.0;
                veloc2[indice] = 0.0;
                veloc3[indice] = 0.0;
                veloc4[indice] = 0.0;
                veloc5[indice] = 0.0;
                veloc6[indice] = 0.0;
                veloc7[indice] = 0.0;
	}

           franka::JointVelocities velocities = {{veloc1[indice], veloc2[indice], veloc3[indice], veloc4[indice], veloc5[indice], veloc6[indice], veloc7[indice]}};
	
		veloc7real[indice] = robot_state.dq[6];

          if (time >= (timemax+0.05)) {
              cout << "Error joint 1: " << (robot_state.q[0]-dataMPC.posgoal[0])*100/dataMPC.posgoal[0] << "%" <<endl;
              cout << "Error joint 2: " << (robot_state.q[1]-dataMPC.posgoal[1])*100/dataMPC.posgoal[1] << "%" <<endl;
              cout << "Error joint 3: " << (robot_state.q[2]-dataMPC.posgoal[2])*100/dataMPC.posgoal[2] << "%" <<endl;
              cout << "Error joint 4: " << (robot_state.q[3]-dataMPC.posgoal[3])*100/dataMPC.posgoal[3] << "%" <<endl;
              cout << "Error joint 5: " << (robot_state.q[4]-dataMPC.posgoal[4])*100/dataMPC.posgoal[4] << "%" <<endl;
              cout << "Error joint 6: " << (robot_state.q[5]-dataMPC.posgoal[5])*100/dataMPC.posgoal[5] << "%" <<endl;
              cout << "Error joint 7: " << (robot_state.q[6]-dataMPC.posgoal[6])*100/dataMPC.posgoal[6] << "%" <<endl;
              cout << robot_state.q[0] << "   " <<robot_state.q[1] << "   " <<robot_state.q[2] << "   " <<robot_state.q[3] << "   " <<robot_state.q[4] << "   " <<robot_state.q[5] << "   " <<robot_state.q[6] << endl;


                                             //   cout  << robot_state.q[6] << "  " << dataMPC.posgoal[6] << endl;

						int index1 =floor(timemax*1000 + 50);


						ofstream myfile("comVEL.csv");
                                        if (myfile.is_open())
		  				{
							for (int k = 0; k < index1; k++){
                                                                        myfile << veloc7[0+k] << "," << veloc7real[0+k] << endl;
								}
		   			 	myfile.close();
							}
            return franka::MotionFinished(velocities);
                }
          return velocities;
      });

   }
   pthread_exit(NULL);
}


int main () {
    dataMPC.T = 25; //solver is set for this prediction horizon - DO NOT MODIFY
    dataMPC.resample = 0.1; //should not be bigger than 0.2
    dataMPC.span = 0.7; // If span>2. some changes should be applied for veloc7 vectors
    double posgoalwrite[7] = {-0.059, -0.450, -0.237, -2.4, -0.136, 1.951, 0.528};
    for(int i = 0; i < 7; i++){
        dataMPC.posgoal[i] = posgoalwrite[i];
    }

    dataMPC.size_res = 500; //size should be, at least, 4000 times resample
    dataMPC.flag = 1;
//if(sizeof(msgMPC) != 8*dataMPC.size_res){
//cout<< "Please check that the size of msgMPC and msgMPC2 are equal to the one defined in the struct dataMPC " << endl;
//exit(-1);
//}
   
   
   pthread_mutex_init(&mtx, NULL);
   pthread_t threads[NUM_THREADS];
   int rc;
   int i;

   for(i = 0; i < NUM_THREADS; i++ ) {
      //cout << "main() : creating thread, " << i << endl;
      rc = pthread_create(&threads[i], NULL, ThreadsDistribution, (void *) (intptr_t)i);

      if (rc) {
         cout << "Error:unable to create thread," << rc << endl;
         exit(-1);
      }
   }
   pthread_exit(NULL);
}

void load_static_data(int joint) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 0;
  params.Q[3] = 0;
  params.Q[6] = 0;
  params.Q[1] = 0;
  params.Q[4] = 1;
  params.Q[7] = 0;
  params.Q[2] = 0;
  params.Q[5] = 0;
  params.Q[8] = 1;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 0.001;
  if(joint == 7){
      params.z_min[0] = -2.5;
      params.z_min[1] = -2.5;
      params.z_min[2] = -18;
      params.z_max[0] = 2.5;
      params.z_max[1] = 2.5;
      params.z_max[2] = 18;
      params.u_min[0] = -7000;
      params.u_max[0] = 7000;
  }
  else if(joint == 1) {
      params.z_min[0] = -2.5;
      params.z_min[1] = -2;
      params.z_min[2] = -10;
      params.z_max[0] = 2.5;
      params.z_max[1] = 2;
      params.z_max[2] = 10;
      params.u_min[0] = -6000;
      params.u_max[0] = 6000;
  }
  else if(joint == 2) {
      params.z_min[0] = -1.56;
      params.z_min[1] = -2;
      params.z_min[2] = -6;
      params.z_max[0] = 1.5;
      params.z_max[1] = 2;
      params.z_max[2] = 6;
      params.u_min[0] = -3000;
      params.u_max[0] = 3000;
  }
  else if(joint == 3) {
      params.z_min[0] = -2.5;
      params.z_min[1] = -2;
      params.z_min[2] = -8;
      params.z_max[0] = 2.5;
      params.z_max[1] = 2;
      params.z_max[2] = 8;
      params.u_min[0] = -4000;
      params.u_max[0] = 4000;
  }
  else if(joint == 4) {
      params.z_min[0] = -2.5;
      params.z_min[1] = -2;
      params.z_min[2] = -10;
      params.z_max[0] = -0.07;
      params.z_max[1] = 2;
      params.z_max[2] = 10;
      params.u_min[0] = -6000;
      params.u_max[0] = 6000;
  }
  else if(joint == 5) {
      params.z_min[0] = -2.5;
      params.z_min[1] = -2;
      params.z_min[2] = -12;
      params.z_max[0] = 2.5;
      params.z_max[1] = 2;
      params.z_max[2] = 12;
      params.u_min[0] = -7000;
      params.u_max[0] = 7000;
  }
  else if(joint == 6) {
      params.z_min[0] = 0;
      params.z_min[1] = -2;
      params.z_min[2] = -18;
      params.z_max[0] = 3.0;
      params.z_max[1] = 2;
      params.z_max[2] = 18;
      params.u_min[0] = -8000;
      params.u_max[0] = 8000;
  }

  //Velocity and acceleration in goal are always 0
  params.goal[1] = 0.0;
  params.goal[2] = 0.0;
}

void model(double h) {
  double Fi[3][3] = {{1, h, h*h/2},{0, 1, h},{0, 0, 1}};
  double Firow[9] = {1, 0, 0, h, 1, 0, h*h/2, h, 1};
  double Gamma1[3] = {h*h*h*h/24, h*h*h/6, h*h/2};
  double Gamma[3] = {h*h*h/6, h*h/2, h};

  for(int i=0; i<9; i++){
    params.A[i] = Firow[i];
  }

  for(int i=0; i<3; i++){
    params.Dz[i] = Gamma1[i]/h;
  }

  params.B[0] = Gamma[0] + ((Fi[0][0] - 1)*Gamma1[0] + Fi[0][1]*Gamma1[1] + Fi[0][2]*Gamma1[2])/h;
  params.B[1] = Gamma[1] + (Fi[1][0]*Gamma1[0] + (Fi[1][1] - 1)*Gamma1[1] + Fi[1][2]*Gamma1[2])/h;
  params.B[2] = Gamma[2] + (Fi[2][0]*Gamma1[0] + Fi[2][1]*Gamma1[1] + (Fi[2][2] - 1)*Gamma1[2])/h;
}

void goal(double posgoal) {
  params.goal[0] = posgoal;
}

void init_z(double zpos, double zvel, double zacel) {
  params.z_0[0] = zpos;
  params.z_0[1] = zvel;
  params.z_0[2] = zacel;
}

void init_x(double xpos, double xvel, double xacel) {
  params.x_0[0] = xpos;
  params.x_0[1] = xvel;
  params.x_0[2] = xacel;
}

void initial_state(double hprev, double resample, int k, int joint, DataMPC dataMPC){

    double x1[26];
    double x2[26];
    double x3[26];
    double z1[26];
    double z2[26];
    double z3[26];


    if(joint == 1){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j1_1[i];
            x2[i] = dataMPC.x_prev_j1_2[i];
            x3[i] = dataMPC.x_prev_j1_3[i];
            z1[i] = dataMPC.z_prev_j1_1[i];
            z2[i] = dataMPC.z_prev_j1_2[i];
            z3[i] = dataMPC.z_prev_j1_3[i];

        }
    }
    else if(joint == 2){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j2_1[i];
            x2[i] = dataMPC.x_prev_j2_2[i];
            x3[i] = dataMPC.x_prev_j2_3[i];
            z1[i] = dataMPC.z_prev_j2_1[i];
            z2[i] = dataMPC.z_prev_j2_2[i];
            z3[i] = dataMPC.z_prev_j2_3[i];

        }
    }
    else if(joint == 3){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j3_1[i];
            x2[i] = dataMPC.x_prev_j3_2[i];
            x3[i] = dataMPC.x_prev_j3_3[i];
            z1[i] = dataMPC.z_prev_j3_1[i];
            z2[i] = dataMPC.z_prev_j3_2[i];
            z3[i] = dataMPC.z_prev_j3_3[i];

        }
    }
    else if(joint == 4){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j4_1[i];
            x2[i] = dataMPC.x_prev_j4_2[i];
            x3[i] = dataMPC.x_prev_j4_3[i];
            z1[i] = dataMPC.z_prev_j4_1[i];
            z2[i] = dataMPC.z_prev_j4_2[i];
            z3[i] = dataMPC.z_prev_j4_3[i];

        };
    }
    else if(joint == 5){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j5_1[i];
            x2[i] = dataMPC.x_prev_j5_2[i];
            x3[i] = dataMPC.x_prev_j5_3[i];
            z1[i] = dataMPC.z_prev_j5_1[i];
            z2[i] = dataMPC.z_prev_j5_2[i];
            z3[i] = dataMPC.z_prev_j5_3[i];

        }
    }
    else if(joint == 6){
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j6_1[i];
            x2[i] = dataMPC.x_prev_j6_2[i];
            x3[i] = dataMPC.x_prev_j6_3[i];
            z1[i] = dataMPC.z_prev_j6_1[i];
            z2[i] = dataMPC.z_prev_j6_2[i];
            z3[i] = dataMPC.z_prev_j6_3[i];

        }
    }
    else{
        for(int i=0; i<26; i++){
            x1[i] = dataMPC.x_prev_j7_1[i];
            x2[i] = dataMPC.x_prev_j7_2[i];
            x3[i] = dataMPC.x_prev_j7_3[i];
            z1[i] = dataMPC.z_prev_j7_1[i];
            z2[i] = dataMPC.z_prev_j7_2[i];
            z3[i] = dataMPC.z_prev_j7_3[i];

        }
    }





	if (k == 0){
                double pos01;
                if(joint == 1){
                    pos01 = 0.0;
                }
                else if(joint == 2){
                    pos01 = -M_PI/4;
                }
                else if(joint == 3){
                    pos01 = 0.0;
                }
                else if(joint == 4){
                    pos01 = -3*M_PI/4;
                }
                else if(joint == 5){
                    pos01 = 0.0;
                }
                else if(joint == 6){
                    pos01 = M_PI/2;
                }
                else{
                    pos01 = 0.0;
                }
                init_z(pos01, 0.0, 0.0);
                init_x(pos01, 0.0, 0.0);
	}
	else{
		int initnext = floor(resample/hprev);
		double diveven = fmod(resample,hprev);
		if (abs(diveven) < 0.00001){
                        init_z(z1[initnext], z2[initnext], z3[initnext]);
                        init_x(x1[initnext], x2[initnext], x3[initnext]);
		}
		else {
                        double z01 = z1[initnext]+diveven*(z1[initnext+1]-z1[initnext])/hprev;
                        double z02 = z2[initnext]+diveven*(z2[initnext+1]-z2[initnext])/hprev;
                        double z03 = z3[initnext]+diveven*(z3[initnext+1]-z3[initnext])/hprev;
                        double x01 = x1[initnext]+diveven*(x1[initnext+1]-x1[initnext])/hprev;
                        double x02 = x2[initnext]+diveven*(x2[initnext+1]-x2[initnext])/hprev;
                        double x03 = x3[initnext]+diveven*(x3[initnext+1]-x3[initnext])/hprev;
			init_z(z01, z02, z03);
			init_x(x01, x02, x03);

		}
	}
}

int send(int internal_flag1, msg msgMPC2){
	while(internal_flag1 == 0){
				pthread_mutex_lock(&mtx);
				if (dataMPC.flag == 1){
 		        		for(int j=0;j<500;j++){
                                                msgMPC.j1[j] = msgMPC2.j1[j];
                                                msgMPC.j2[j] = msgMPC2.j2[j];
                                                msgMPC.j3[j] = msgMPC2.j3[j];
                                                msgMPC.j4[j] = msgMPC2.j4[j];
                                                msgMPC.j5[j] = msgMPC2.j5[j];
                                                msgMPC.j6[j] = msgMPC2.j6[j];
                                                msgMPC.j7[j] = msgMPC2.j7[j];
					}
					internal_flag1 = 1;
					dataMPC.flag = 0;
				}
				pthread_mutex_unlock(&mtx);
				usleep(5000);
	}
	return internal_flag1;
}

int receive_first(int internal_flag2, double veloc1[200], double veloc2[200], double veloc3[200], double veloc4[200], double veloc5[200], double veloc6[200], double veloc7[200], msg msgMPC){
	while(internal_flag2 == 0){
                        pthread_mutex_lock(&mtx);

			for(int j=0;j<200;j++){
                            veloc1[j] = msgMPC.j1[j];
                            veloc2[j] = msgMPC.j2[j];
                            veloc3[j] = msgMPC.j3[j];
                            veloc4[j] = msgMPC.j4[j];
                            veloc5[j] = msgMPC.j5[j];
                            veloc6[j] = msgMPC.j6[j];
                            veloc7[j] = msgMPC.j7[j];
			}
			dataMPC.flag = 1;
			internal_flag2 = 1;
	
			pthread_mutex_unlock(&mtx);
	}
	return internal_flag2;
}

int receive(int internal_flag2, double veloc1[200], double veloc2[200], double veloc3[200], double veloc4[200], double veloc5[200], double veloc6[200], double veloc7[200], int indice, msg msgMPC){
	while(internal_flag2 == 0){
		pthread_mutex_lock(&mtx);
		for(int j=0;j<200;j++){
                    veloc1[indice+j] = msgMPC.j1[j];
                    veloc2[indice+j] = msgMPC.j2[j];
                    veloc3[indice+j] = msgMPC.j3[j];
                    veloc4[indice+j] = msgMPC.j4[j];
                    veloc5[indice+j] = msgMPC.j5[j];
                    veloc6[indice+j] = msgMPC.j6[j];
                    veloc7[indice+j] = msgMPC.j7[j];
		}
		dataMPC.flag = 1;
		internal_flag2 = 1;
		pthread_mutex_unlock(&mtx);
	}
	return internal_flag2;
}


void store_next_initial(int joint, Vars vars){


    if(joint == 1){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j1_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j1_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j1_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j1_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j1_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j1_3[i] = *(vars.z[i]+2);
        }
    }
    else if(joint == 2){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j2_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j2_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j2_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j2_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j2_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j2_3[i] = *(vars.z[i]+2);
        }
    }
    else if(joint == 3){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j3_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j3_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j3_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j3_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j3_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j3_3[i] = *(vars.z[i]+2);
        }
    }
    else if(joint == 4){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j4_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j4_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j4_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j4_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j4_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j4_3[i] = *(vars.z[i]+2);
        }
    }
    else if(joint == 5){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j5_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j5_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j5_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j5_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j5_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j5_3[i] = *(vars.z[i]+2);
        }
    }
    else if(joint == 6){
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j6_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j6_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j6_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j6_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j6_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j6_3[i] = *(vars.z[i]+2);
        }
    }
    else{
        for(int i=1; i<26; i++){
            dataMPC.x_prev_j7_1[i] = *(vars.x[i]);
            dataMPC.x_prev_j7_2[i] = *(vars.x[i]+1);
            dataMPC.x_prev_j7_3[i] = *(vars.x[i]+2);
            dataMPC.z_prev_j7_1[i] = *(vars.z[i]);
            dataMPC.z_prev_j7_2[i] = *(vars.z[i]+1);
            dataMPC.z_prev_j7_3[i] = *(vars.z[i]+2);
        }
    }



}


