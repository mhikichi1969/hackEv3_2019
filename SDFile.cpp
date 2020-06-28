#include "SDFile.h"
#include "ev3api.h"
#include "util.h"

SDFile::SDFile()
{

}

bool SDFile::open(char *path)
{
    memfile_t memfile;
    ER sd_err;


    sd_err = ev3_memfile_load(path,&memfile);

    char buf[256];
    sprintf(buf,"SDOPEN %d",sd_err);
    msg_f(buf,9);

    if(sd_err!=E_OK) {
        msg_f("SD_ERR",10);
        ev3_speaker_play_tone(NOTE_C4,40);
        return false;
    }

    mMemFile = (char*)memfile.buffer;
    current_pt = next_pt = mMemFile;


    return true;
}

void SDFile::current_read(int len)
{
    int data;
    bool minus=false;
    

    char ch = *current_pt;
    char buf[256];

    while(cur_len<len) {
        for(int i=0;i<3;i++) {
            minus=false;

            if(ch=='-') {
                minus = true;
                current_pt++;
            }
            int num=0;
            ch=*current_pt;

            while(ch!=',' && ch!='\r' && ch!='\n') {
                num = num*10;
                num += ch-'0';
                current_pt++;
                ch=*current_pt;
                //sprintf(buf,"%d   %d %d %d",num , *(current_pt-1),ch,*(current_pt+1));
                //msg_f(buf,9);
            }
            if(minus) num*=-1;
            line[i] = num;
            current_pt++; // , と　改行飛ばし
            if(*current_pt=='\n')
                current_pt++;  // LF飛ばし
            ch = *current_pt;
        }
        cur_len=line[0];
        cur_L=line[1];
        cur_R=line[2];
        if(cur_len==99999) {
            msg_f("data_end",12);
            break;
        }
    }
  //  sprintf(buf,"%d,%d,%d",cur_len,cur_L,cur_R);
  //  msg_f(buf,10);
}

void SDFile::next_read(int len)
{
    int data;
    bool minus=false;

    char ch = *next_pt;
    
    while(next_len<len) {
        minus=false;
        for(int i=0;i<3;i++) {
            if(ch=='-') {
                minus = true;
                next_pt++;
            }
            int num=0;
            ch=*next_pt;

            while(ch!=',' &&  ch!='\r' &&  ch!='\n') {
                num = num*10;
                num += ch-'0';
                next_pt++;
                ch=*next_pt;
            }
            if(minus) num*=-1;
            line[i] = num;
            next_pt++; // , と　改行飛ばし
            if(*next_pt=='\n')
                next_pt++;  // LF飛ばし
            ch = *next_pt;
       }
        next_len=line[0];
        next_L=line[1];
        next_R=line[2];
        if(next_len==99999) {
            msg_f("data_end",13);
            break;
        }

    }

  //  char buf[256];
  //  sprintf(buf,"%d,%d,%d",next_len,next_L,next_R);
  //  msg_f(buf,11);

}

double SDFile::getCurveParam()
{
    if(next_R-cur_R==0) return 1;
    return (double)(next_L-cur_L)/(next_R-cur_R);
}