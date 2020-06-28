#ifndef _SDFILE_H_
#define _SDFILE_H_
class SDFile
{
    public:
    SDFile();

    bool open(char *path);
    void current_read(int len);
    void next_read(int len);
    double getCurveParam();

    private:
    char *mMemFile;
    char *current_pt;
    char *next_pt;

    int line[3] = {-100,-100,-100};
    int cur_len=-100;
    int next_len=-100;
    int cur_L;
    int next_L;
    int cur_R;
    int next_R;

};

#endif
