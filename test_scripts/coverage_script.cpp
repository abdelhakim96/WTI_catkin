#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <string>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iterator>
#include <numeric>
using namespace std;
//float alpha_t=30;
//float alpha_d=0;
//float gamma_d=30;
std::vector<float>  lv ={};
float T0 [2]={0, 0};
float T1 [2];
float T2 [2];
float L = 3;
float x0 [2]={-2, -2};
float l;

float d_a[4];

float a0[2];
float a1[2];
float a2[2];
float la1;
float la2;


float t0[2];
float t1[2];
float t2[2];


float P0 [2];
float P1 [2];
float P2 [2];

float mt;
float m0;
float m1;
float m2;
float ct;
float c0;
float c1;
float c2;
float alpha_0;
float alpha_1;
float alpha_2;
float d_P1T1;
float d_P1T2;
float d_P2T1;
float d_P2T2;
float a0t0;
float a1t1;
float a2t2;
float angle[50];
float length[11][11][37];
float maxx;
float minx;
float maxy;
float miny;
float psi;
float px;
float py;
float l1;
float l2;
int cc;
float a_t;
float temp [2];




 
int main(int argc, char **argv) {
c0=0;
c1=0;
c2=0;
l=0;
l1=0;
l2=0;
psi=0;
a_t=0.0001;
int count=0;
//for (int p = 0; p < 11; p++){
   
//for (int q = 0; q < 11; q++){
//for (int r = 0; r < 37; r++){
  
#include <fstream>
std::ifstream infile("myData_1.txt");

float a, b, c, d,e;

cc=1;
while (infile >> a >> b >> c >> d>>e)
{
    // process pair (a,b)


  for  (int ang = 1; ang < 3; ang++){

  if (ang==1) {
     a_t=0.0001;
  }  

  else
  {
     a_t=90.001;
  }
  psi=a;
  px=b;
  py=c;
  x0[0]=d;
  x0[1]=e;

  count=count+1;
 // x0[0]=0;
 // x0[1]=0;

//x0[1]=x0[1];





psi=psi+90;
psi=psi*(3.142/180);
float alpha_t=a_t*(3.142/180);  //degree to rad
float gamma_d=40*(3.142/180);
float alpha_d=psi;
//psi=psi+1.9;
T0[0]=px;
T0[1]=py;

//calculate end points of Target 
T1[0]=T0[0]+(L/2)*sin(alpha_t);   
T1[1]=T0[1]+(L/2)*cos(alpha_t);

T2[0]=T0[0]-(L/2)*sin(alpha_t);
T2[1]=T0[1]-(L/2)*cos(alpha_t);


 //calculate equation of the line of the target  
mt=(T2[1]-T1[1])/((T2[0]-T1[0])+0.00000001);
ct=T2[1]-mt*T2[0];


 //calculate equation of the line of the incident rays l1 and l2 and L0



alpha_1=alpha_d+(gamma_d/2);
alpha_2=alpha_d-(gamma_d/2);
alpha_0=alpha_d;

m0=1/tan(alpha_0+0.00001);
m1=1/tan(alpha_1);
m2=1/tan(alpha_2);

c0=x0[1]-m0*x0[0];

c1=x0[1]-m1*x0[0];
c2=x0[1]-m2*x0[0];


// find intersection points P1 and P2 and P0

P0[0]=(ct-c0)/(m0-mt);
P0[1]=m0*P0[0]+c0;

P1[0]=(ct-c1)/(m1-mt);
P1[1]=m1*P1[0]+c1;

P2[0]=(ct-c2)/(m2-mt);
P2[1]=m2*P2[0]+c2;


 //Calculate the distance between intersection points an end points of the target


d_P1T1=pow((pow(P1[0]-T1[0],2)+pow(P1[1]-T1[1],2)),0.5);
d_P1T2=pow((pow(P1[0]-T2[0],2)+pow(P1[1]-T2[1],2)),0.5);
d_P2T1=pow((pow(P2[0]-T1[0],2)+pow(P2[1]-T1[1],2)),0.5);
d_P2T2=pow((pow(P2[0]-T2[0],2)+pow(P2[1]-T2[1],2)),0.5);



d_a[0]=d_P1T1;
d_a[1]=d_P1T2;
d_a[2]=d_P2T1;
d_a[3]=d_P2T2;


float max=d_a[0];
// Check the distance for correct points ordering

int n=4; 

for (int i = 1; i < n; i++){
        if (d_a[i] > max) 
        {
            max = d_a[i];
        }
}


float tempx;
float tempy;
if (max ==d_P2T2 || max==d_P1T1)
   {tempx=P1[0];
   P1[0]=P2[0];
   P2[0]=tempx;
   tempy=P1[1];
   P1[1]=P2[1];
   P2[1]=tempy;
   }


// Calculate a1 and a2 , t1 and t2



a0[0]=x0[0]+sin(alpha_0);
a0[1]=x0[1]+cos(alpha_0);

a1[0]=P1[0]-T1[0];
a1[1]=P1[1]-T1[1];



a2[0]=P2[0]-T2[0];
a2[1]=P2[1]-T2[1];

la1=pow((pow(a1[0],2)+pow(a1[1],2)),0.5);
la2=pow((pow(a2[0],2)+pow(a2[1],2)),0.5);



t0[0]=P0[0]-x0[0];
t0[1]=P0[1]-x0[1];

t1[0]=T2[0]-T1[0];
t1[1]=T2[1]-T1[1];

t2[0]=T1[0]-T2[0];
t2[1]=T1[1]-T2[1];

a0t0= a0[0]*t0[0]+a0[1]*t0[1];
a1t1= a1[0]*t1[0]+a1[1]*t1[1];
a2t2= a2[0]*t2[0]+a2[1]*t2[1];


// Calculate the viewed length l


if (T1[0]>T2[0])
{
   maxx=T1[0];
   minx=T2[0];
}
else{
   maxx=T2[0];
   minx=T1[0];

}

if (T1[1]>T2[1])
{
   maxy=T1[1];
   miny=T2[1];
}
else{
   maxy=T2[1];
   miny=T1[1];

}

if (miny<P1[1]&&P1[1]<maxy && minx<P1[0]&&P1[0]<maxx){
    

   if (miny<P2[1]&&P2[1]<maxy && minx<P2[0]&&P2[0]<maxx){

     l=pow((pow(P1[0]-P2[0],2)+pow(P1[1]-P2[1],2)),0.5);
      cout <<"4" <<endl;
   

   }
   else{ 

      l=pow((pow(P1[0]-T2[0],2)+pow(P1[1]-T2[1],2)),0.5);
        cout <<"3 " <<endl;
     }

}

else{
   if (miny<P2[1]&&P2[1]<maxy && minx<P2[0]&&P2[0]<maxx){
      l=pow((pow(P2[0]-T1[0],2)+pow(P2[1]-T1[1],2)),0.5);
      cout <<"2 " <<endl;
   }

   else{

      if (a1t1*a2t2>0 )
   { 

      l=pow((pow(T2[0]-T1[0],2)+pow(T2[1]-T1[1],2)),0.5);
      cout <<"1 " <<endl;

   }


}



}
/*
cout << "x " << x0[0] <<endl;
cout << "y " << x0[1] <<endl;
cout <<"psi "<< psi*(180/3.142) <<endl;
cout <<"length "<<  l <<endl;
cout <<"counter "<<  cc <<endl;
cout <<"T0x "<< T0[0] <<endl;
cout <<"T0y "<< T0[1] <<endl;
cout <<"T1x "<<  T1[0] <<endl;
cout <<"T1y "<<  T1[1] <<endl;
cout <<"T2x "<<  T2[0] <<endl;
cout <<"T2y "<<  T2[1] <<endl;
cout <<"P0x "<<  P0[0] <<endl;
cout <<"P0y "<<  P0[1] <<endl;
cout <<"P1x "<<  P1[0] <<endl;
cout <<"P1y "<<  P1[1] <<endl;
cout <<"P2x "<<  P2[0] <<endl;
cout <<"P2y "<<  P2[1] <<endl;
cout <<"ct "<<  ct <<endl;
cout <<"mt "<<  mt <<endl;
cout <<"m0 "<<  m0 <<endl;
cout <<"c0 "<<  c0 <<endl;
*/
if (ang==1)
{
l1=l;
}

else{
   l2=l;
   if(l2>l1)
   {
      l=l2;
  }
   else{
      l=l1;
   }
 

   lv.push_back(l);
    ofstream output_file( "hakim.txt" );
    ostream_iterator<float> output_iterator( output_file, "\n" );
    // Passing all the variables inside the vector from the beginning of the vector to the end.
    copy( lv.begin( ), lv.end( ), output_iterator );
}



  //cout << count <<endl;

//length[p][q][r]=l;

//}
//}
//}
cc=cc+1;

}
}

}






