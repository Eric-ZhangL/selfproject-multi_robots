#include <common/public.h>

vector<string> getFileList(char *basePath,char *sep)
{
    DIR *dir;
    struct dirent *ptr;
    vector<string> filelist;

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
    }

    filelist.clear();
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            vector<string> ss=split(ptr->d_name,".");
            if(ss.size()>1 && ss[ss.size()-1]==sep) filelist.push_back(ss[0]);
        }
        else if(ptr->d_type == 10)   //link file
        {
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if(ptr->d_type == 4)    ///dir
        {
        }
    }
    closedir(dir);

    return filelist;
};

void GetPackagePath(char *packname,char *path)
{
    char cmd[100];
    sprintf(cmd,"rospack find %s",packname);
    FILE *fp=popen(cmd,"r");

    fgets(path, 256, fp);
    pclose(fp);
    //printf("%s\n",path);
}

vector<string> split(const string& s, const string& sep)
{
    vector<string> v;
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
};


LS_Coeff Least_Square(vector<geometry_msgs::Point32> point)
{
    LS_Coeff A={0,0,0};

    if(point.size()==2) 
    {
        A.a1=(point[1].y-point[0].y)/(point[1].x-point[0].x);
        A.a0=point[1].y-A.a1*point[1].x;
        return A;
    }

    double sumX = 0, sumY = 0, sumXX = 0, sumXXX = 0, sumXXXX = 0, sumXY = 0, sumXXY = 0;
    for(int i=0;i<point.size();i++)
    {
        sumX=sumX+point.at(i).x;
        sumY=sumY+point.at(i).y;
        sumXX=sumXX+point.at(i).x*point.at(i).x;
        sumXXX=sumXXX+point.at(i).x*point.at(i).x*point.at(i).x;
        sumXXXX=sumXXXX+point.at(i).x*point.at(i).x*point.at(i).x*point.at(i).x;
        sumXY=sumXY+point.at(i).x*point.at(i).y;
        sumXXY=sumXXY+point.at(i).x*point.at(i).x*point.at(i).y;
    }
    double a11 = point.size();   double  a12 = sumX;   double  a13 = sumXX;   double  b1 = sumY;
    double  a21 = sumX;          double  a22 = sumXX;  double  a23 = sumXXX;  double  b2 = sumXY;
    double  a31 = sumXX;         double  a32 = sumXXX; double  a33 = sumXXXX; double  b3 = sumXXY;

    double a21x = a21 - a21 / a11*a11;
    double a22x = a22 - a21 / a11*a12;
    double a23x = a23 - a21 / a11*a13;
    double b2x = b2 - a21 / a11*b1;

    double a31x = a31 - a31 / a11*a11;
    double  a32x = a32 - a31 / a11*a12;
    double  a33x = a33 - a31 / a11*a13;
    double b3x = b3 - a31 / a11*b1;

    double a32xx = a32x - a32x / a22x*a22x;
    double  a33xx = a33x - a32x / a22x*a23x;
    double  b3xx = b3x - a32x / a22x*b2x;

    A.a2=b3xx / a33xx;
    A.a1=(b2x - a23x*A.a2) / a22x;
    A.a0=(b1 - A.a1*a12 - A.a2*a13) / a11;
    return A;
};

vector<string> readFileList(char *basePath)
{
    DIR *dir;
    struct dirent *ptr;
    vector<string> filelist;

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
    }

    filelist.clear();
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            filelist.push_back(ptr->d_name);
        }
        else if(ptr->d_type == 10)    ///link file
        {
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if(ptr->d_type == 4)    ///dir
        {
        }
    }
    closedir(dir);

    return filelist;
}

int FindMinID(vector<float> datalist)
{
    if(datalist.size()==0) return -1;

    vector<float>::iterator itMin = min_element(datalist.begin(), datalist.end());
    return distance(datalist.begin(), itMin);
}

#define HIG_UINT16(a) (((a) >> 8) & 0xFF)
#define LOW_UINT16(a) ((a)&0xFF)
#define HIG_UINT8(a) (((a) >> 4) & 0x0F)
#define LOW_UINT8(a) ((a)&0x0F)

uint16_t crctalbeabs[] =
    {
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
        0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400};

uint16_t CRC16(uint8_t *ptr, uint32_t len)
{
    uint16_t crc = 0xffff;
    uint32_t i;
    uint8_t ch;

    for (i = 0; i < len; i++)
    {
        ch = *ptr++;
        crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
        crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
    }

    return crc;
}