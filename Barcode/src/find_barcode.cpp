#include "find_barcode.h"
float pic_width = 640;
float pic_height = 480;
string WINDOW= "barcode";
Mat src;
float barcode_width = 0.0535,barcode_height = 0.019;
Point3f p1(-barcode_width/2,0,barcode_height);
Point3f p2(-barcode_width/2,0,0);
Point3f p3(+barcode_width/2,0,barcode_height);
Point3f p4(barcode_width/2,0,0);
ofstream OutFile;
ofstream manualFile;
vector<Point2f> uv_manul;
Point2f ul_point;

int num_picture;  //要分析的图片数量
vector<cv::Point3f>  P_vector;
Mat K = Mat::eye(3,3,CV_32F);
void para_initialize(int mode){
    K.at<float>(0,0) = 615.31;
    K.at<float>(1,1) = 615.66;
    K.at<float>(0,2) = 326.29;
    K.at<float>(1,2) = 247.49;
    P_vector.push_back(p1);
    P_vector.push_back(p2);
    P_vector.push_back(p3);
    P_vector.push_back(p4);
    if(mode ==2)
        OutFile.open("自动标定结果/auto.txt",ios::app);
    else
        manualFile.open("手动标定结果/manul.txt",ios::app);  //根据需要选择打开
}

void task_over(){
    if(OutFile.is_open())
        OutFile.close();
    if(manualFile.is_open())
        manualFile.close();   //根据需要来关闭
}

vector<string> getFiles(string cate_dir)
{   
    cout<<cate_dir<<endl;
    vector<string> files,files_jpg;//存放文件名
    DIR *dir; struct
    dirent *ptr;
    if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
        perror("Open dir error...");
        exit(1);
        }
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
         continue;
        else if(ptr->d_type == 8) ///file //
        files.push_back(ptr->d_name);
        else if(ptr->d_type == 10) ///link file //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
               }
    }
    closedir(dir);
    string temp;
    for (string s: files ){
        if (s[s.length()-1] == 'g'){
            temp = s.substr(0,s.length()-4);
            files_jpg.push_back(temp);
        }
    }
    return files_jpg;
}


void On_mouse(int event, int x, int y, int flags, void*)//每次点击左键，将将当前点坐标存储到txt文件中，并在相应位置画红点
{
    if (event == EVENT_LBUTTONDOWN && uv_manul.size()<4 )
    {
        Point2f recent_Point = Point2f(x, y);
        recent_Point.x = recent_Point.x+ul_point.x;
        recent_Point.y = recent_Point.y+ul_point.y;
        manualFile << recent_Point.x << " " << recent_Point.y<<endl;
        uv_manul.push_back(recent_Point);
        circle(src, recent_Point - ul_point, 5, Scalar(0, 0, 255), -1);
        imshow(WINDOW, src);
        if(uv_manul.size()==4) {  //点击数量达到4个以后,按任意键切换到下一个图片
            Mat R,t,r;
            solvePnP(P_vector,uv_manul,K,Mat(), r,t,false,cv::SOLVEPNP_EPNP);
            Rodrigues(r,R);
            cout<<"旋转矩阵:"<<endl<<R<<endl;
            cout<<"平移矩阵:"<<endl<<t<<endl;
            manualFile<<"旋转矩阵:"<<endl<<R<<endl;
            manualFile<<"欧拉角:"<<rotationMatrixToEulerAngles(R)<<endl;
            manualFile<<"平移矩阵:"<<endl<<t<<endl;
            manualFile<<"相机中心坐标:"<<endl<<-R.t()*t<<endl;
        }
    }
}

void find_manual(Mat img){
    namedWindow(WINDOW, WINDOW_NORMAL);
    setMouseCallback(WINDOW, On_mouse);
    src = img.clone();
    imshow(WINDOW, src);//点击的开始
    waitKey(0);
    return ;
}


void readTxt(string file , vector<barcode_position>& barcode_positions)
{
    ifstream infile;

    infile.open(file.data());   //将文件流对象与文件连接起来
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
    string s;
    vector<float> res;
    barcode_position barcode;
    while(getline(infile,s))
    {
        if(s[0] == '0'){
            res.clear();

            while (!s.empty())
            {
                if (s.find(" ") == string::npos)
                {
                    res.push_back(stof(s));
                    s.clear();
                    break;
                }
          string s_temp = s.substr(0, s.find(" "));
          res.push_back(stof(s_temp));
          s.erase(0, s.find(" ") + 1);
            }
            barcode.center = Point2f(res[2],res[3]);
            barcode.width = res[4]*1.2;
            barcode.height = res[5]*1.2;
            float up_left_x = max(10.0f,barcode.center.x -  barcode.width/2);
            float up_left_y = max(10.0f,barcode.center.y -  barcode.height/2);
            float down_right_x = min((barcode.center.x +  barcode.width/2),pic_width);
            float down_right_y = min(barcode.center.y +  barcode.height/2,pic_height);
            barcode.center = Point2f((up_left_x+down_right_x)/2,(up_left_y+down_right_y)/2);
            barcode.width = -up_left_x+down_right_x;
            barcode.height = -up_left_y+down_right_y;

            barcode_positions.push_back(barcode);
        }
    }
    infile.close();             //关闭文件输入流
}

void handlebarcode(barcode_position l, Mat img ){
    cv::Mat image = img.clone() ;
    Mat ROI;
    Point2f up_left(l.center.x-l.width/2,l.center.y-l.height/2);
    up_left.x = max(1.0f,up_left.x);
    up_left.y = max(1.0f,up_left.y);
    Rect rect(up_left.x,up_left.y, l.width, l.height);
    ROI = image(rect).clone();
    vector<Point2f> p;
    barcode_localization(ROI,p);

    for (int i=0;i<p.size();i++){
        p[i] = p[i] + up_left;
        OutFile<<p[i]<<endl;
    }

    for (auto it:p){
       cout<<it<<endl;
    }

    Mat R,t,r;
    solvePnP(P_vector,p,K,Mat(),r,t,false,cv::SOLVEPNP_EPNP);
    Rodrigues(r,R);
    cout<<"旋转矩阵:"<<endl<<R<<endl;
    cout<<"平移矩阵:"<<endl<<t<<endl;
    OutFile<<"旋转矩阵:"<<endl<<R<<endl;
    OutFile<<"欧拉角:"<<rotationMatrixToEulerAngles(R)<<endl;
    OutFile<<"平移矩阵:"<<endl<<t<<endl;
    OutFile<<"相机中心坐标:"<<endl<<-R.t()*t<<endl;
}


void handlephoto (string p){
    string file_path;
    cv::Mat image ;
    vector<barcode_position> barcode_positions;
    barcode_positions.clear();
    file_path=p + (".jpg");
    image = imread(file_path);
    readTxt(p+".txt",barcode_positions);       //得到条形码的位置信息;
    OutFile<<"图片编号:"<< p[p.size()-1]<<endl;
    cout<<"图片编号:"<< p[p.size()-1]<<endl;
    for (barcode_position l: barcode_positions){
     handlebarcode(l, image);
    }
    waitKey(0);
}

void manual_handlephoto (string p){
    string file_path;
    cv::Mat image ;
    file_path=p + (".jpg");
    image = imread(file_path);
    manualFile<<"图片编号"<< p[p.size()-1]<<endl;
    vector<barcode_position> barcode_positions;
    barcode_positions.clear();
    readTxt(p+".txt",barcode_positions);
    for (barcode_position l: barcode_positions){
        cv::Mat img = image.clone() ;
        Mat ROI;
        Point2f up_left(l.center.x-l.width/2,l.center.y-l.height/2);
        up_left.x = max(1.0f,up_left.x);
        up_left.y = max(1.0f,up_left.y);
        ul_point = up_left;
        Rect rect(up_left.x,up_left.y, l.width, l.height);
        ROI = img(rect).clone();
        uv_manul.clear();
        find_manual(ROI);
    }
}
