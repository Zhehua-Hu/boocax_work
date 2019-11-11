本工程用QT编写,需要安装opencv3,如想可视化,还需要安装pangolin;

条形码定位,并用P3P计算出相机位姿程序,流程如下:
读取图像--->二值化--->像素反转--->膨胀--->腐蚀--->寻找最大轮廓--->外包矩形近似--->获取矩形定点--->顶点坐标精确化--->P3P计算位姿;
问题: 顶点坐标精确化比较难,目前还没做到这一点;

手动或自动标定条形码顶点位置:见程序 test_barcode.cpp

数据集:image下的两个压缩文件v2_320_IMG.tar.gz 和  v2_TXT.tar.gz,分别对应原始图片以及提取的条形码方框位置,txt文件每行有6个数,分别代表
类别(0代表条形码), 信任度 , 识别框中心位置x,识别框中心位置y,识别框宽,识别框高

依赖库:
opencv3


运行步骤:
首先将工程克隆到自己的目标文件夹
git clone https://github.com/oym1994/boocax_work
注意,无法单独克隆该项目,必须克隆整个工程项目

到工程目录新建build文件夹并编译
cd boocax_work/Barcode
mkdir build
cmake ..
make

运行

./bin/example/test_barcode
若是自己的数据集
./bin/example/test_barcode path    path指代数据集目录,必须包含图片以及条形码位置信息的txt文件,注意要加"",如 "/home/data"

