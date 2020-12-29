#include "goc_pin.h"

int n;//so thu tu ngay trong nam

int PALT,PATT,TALT,TATT;
//PALT, PATT goc cuc ly thuyet, thuc te(dong-tay)
//TALT, TATT goc nghieng ly thuyet, thuc te(bac-nam)
//so ngay
int so_ngay(int year, int month, int date)
 {
        if (month < 3) {
            year--;
            month += 12;
        }
     return (365*year + year/4 - year/100 + year/400 + (153*month - 457)/5 + date - 306);      
  }
 
//tinh goc xoay tam pin
float goc_pin(int year, int month, int date,int hour, int min, int sec,float VD,float KD)
{
	n=so_ngay(year,month,date)-so_ngay(year,1,1)+1;
  float w=2*3.141592654*(n-1)/365;
  float xichma= 57.296*(0.006918-0.399912*cos(w)+0.070257*sin(w)-0.006758*cos(2*w)+0.000907*sin(2*w)-0.002697*cos(3*w)+0.001480*sin(3*w));
  float ET=229.18*(0.000075+0.001868*cos(w)-0.032077*sin(w)-0.014615*cos(2*w)-0.04089*sin(2*w));
  TALT=90-fabs(VD-xichma);
  float localnoon=12-ET/60.0;//thoi gian gui ngay(h)
  float localtime=hour+min/60.0+sec/3600.0-7+KD/15.0;//thoi gian tuc thoi
  PALT=90-(localnoon-localtime)*15;
  float DL=fabs(2.0/15*acos(-tan(VD)*tan(xichma)));//thoi gian ban ngay(h)
	float st_time=localnoon-DL/2.0;//thoi gian mat troi moc(h)
	float spt_time=localnoon+DL/2.0;//thoi gian mawt troi lan(h)
}

int get_n(){return n;}
int get_PA(){return PALT;}
int get_TA(){return TALT;}
