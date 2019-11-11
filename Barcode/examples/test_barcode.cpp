#include "find_barcode.h"
#if test==2
//测试pnp的程序
//handlephoto()  利用条形码定点检测程序自动寻找四个定点;
//manual_handlephoto() 手动标出顶点.

int main(int argc, char** argv){
    string path;  //图片和识别结果存储路径
    if(argc ==2){
        path = argv[1];
    }
    else
        path = "../img/initial_picture";  //图片和识别结果存储路径
    cout<<"数据集路径:"<<path<<endl;
    vector<string> files=getFiles(path);  //得到路径下所有的文件名
    cout<<"请输入模式,1代表手动标定,2代表自动标定,然后按ENTER键结束"<<endl;
    int mode;
    cin>>mode;

    if(mode ==1){
        para_initialize(mode);
        cout<<"您选择了手动标定模式,请按"
              "左上-->左下-->右上-->右下"
              "的顺序依次点击图中条形码的顶点,每次点击4个顶点后,会自动跳到下一张"<<endl;
        for (string p:files){
            manual_handlephoto(path+'/'+p);
        }
        cout<<"任务结束,结果存储在可执行文件的目录 '/自动标定结果' 目录下"<<endl;
    }
    else if(mode ==2){
        cout<<"您选择了自动标定模式,在显示的图片上按空格键转到下一张图片"<<endl;
        para_initialize(mode);
        for (string p:files){
         handlephoto(path+'/'+p);
            }
        cout<<"任务结束,结果存储在可执行文件的目录 '/手动标定结果' 目录下"<<endl;

    }

    else
        cout<<"输入错误,任务结束"<<endl;
    task_over();
    waitKey(0);
}
#endif
