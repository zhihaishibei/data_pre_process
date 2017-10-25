#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#define foreach BOOST_FOREACH
using namespace std;
using namespace cv;

class DataPreProcess
{
public:
    DataPreProcess();
    bool extract_file(string bag_name, vector<string>topics);
    bool get_sychronise_data();
    void select_data();
    bool make_data_dir(string dir_name);
    bool get_current_dir(string& current_dir );
private:
    int img_size;
    string data_dir;
    string image_dir;
};

DataPreProcess::DataPreProcess()
{
    //make data dir if not exist
    img_size=0;
    string current_dir;
    get_current_dir(current_dir);
    data_dir=current_dir+"data_extracted/";
    make_data_dir(data_dir);
    image_dir=current_dir+"data_extracted/"+"image/";
    make_data_dir(image_dir);
}
bool DataPreProcess::extract_file(string bag_name, vector<string> topics)
{
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int id_img=0;
    int id_pose=0;
    fstream fs((data_dir+"odom.txt").c_str(),ios::out|ios::app);
    if(!fs)
    {
        cout<<"open odom file failed!!!"<<endl;
        return -1;

    }
    fstream fs1((data_dir+"image_time.txt").c_str(),ios::out|ios::app);
    if(!fs1)
    {
        cout<<"open image_time.txt failed"<<endl;
        return -1;
    }
    fstream fs2((data_dir+"data.txt").c_str(),ios::out|ios::app);
    if(!fs2)
    {
        cout<<"open data.txt failed!!"<<endl;
        return -1;
    }
    vector<double> img_stamp;
    vector<double> pose_stamp;

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if(s != NULL)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(s);
            //cout<<cv_ptr->header.stamp<<endl;
            cv::Mat img = cv_ptr->image;
            stringstream ss;
            string name;
            ss<<id_img;
            ss>>name;
            double temp_stamp = s->header.stamp.toNSec();
            fs1<<id_img<<" " <<std::fixed<< temp_stamp<<endl;
            img_stamp.push_back(temp_stamp);
            imwrite(image_dir+name+".jpg",img);
            id_img++;
        }
        nav_msgs::Odometry::ConstPtr i = m.instantiate<nav_msgs::Odometry>();
        if(i!=NULL )
        {
            double temp_stamp= i->header.stamp.toNSec();
            //id_pose<<"       "
              //<<std::fixed<<temp_stamp<<" "
            fs<<i->pose.pose.position.x<<" " 
              <<i->pose.pose.position.y<<" "
              <<i->pose.pose.position.z<<" "
              <<i->pose.pose.orientation.x<<" "
              <<i->pose.pose.orientation.y<<" "
              <<i->pose.pose.orientation.z<<" "
              <<i->pose.pose.orientation.w<<" "<<endl;
            pose_stamp.push_back(temp_stamp);
            id_pose++;
           // cout<<i->header.stamp<<endl;
        }
    }

    for(int i=0;i!=img_stamp.size();i++)
    {
        int j=0;
        while(j!=pose_stamp.size())
        {
            double front = abs(img_stamp[i]-pose_stamp[j]);
            double behind = abs(img_stamp[i]-pose_stamp[j+1]);
            if(front>behind)
                j++;
            else
                break;
        }
        fs2<<i<<" "<<j<<endl;
    }

    img_size=img_stamp.size();


    fs.close();
    fs1.close();
    fs2.close();
    bag.close();
    cout<<"extrct file finished!"<<endl;
    return true;

}

bool DataPreProcess::get_sychronise_data()
{
    ifstream ifs2((data_dir+"data.txt").c_str());
    ofstream ofs((data_dir+"odom_syn.txt").c_str(),ios::app);
    string data_line;
    while(getline(ifs2,data_line))
    {
        stringstream ss;
        ss<<data_line;
        int img_name,odom_seq;
        ss>>img_name>>odom_seq;
        int i=0;
        string odom_line;
        ifstream ifs1((data_dir+"odom.txt").c_str());
        while(i!=odom_seq)
        {
            getline(ifs1,odom_line);
            i++;
        }
        getline(ifs1,odom_line);
        ifs1.close();
        ofs<<odom_line<<endl;
    }
    ifs2.close();
    ofs.close();
    cout<< "synchro finished!"<<endl;
    return true;
}

void DataPreProcess::select_data()
{
    ofstream ofs((data_dir+"image_name.txt").c_str(),ios::app);
    for(int i=0;i!=img_size;i++)
    {
        stringstream ss;
        ss<<i;
        string name;
        ss>>name;

        string image_name=image_dir+name+".jpg";
        Mat img=imread(image_name);
        imshow("image",img);
        char key=waitKey(0);
        if(key=='y')
        {
            ofs<<image_name<<endl;
        }

    }
}

bool DataPreProcess::make_data_dir(string dir_name)
{
    int file_exist=access(dir_name.c_str(), F_OK);
    if(file_exist==-1)
    {
        int mkdir_res=mkdir(dir_name.c_str(),S_IRWXU|S_IRWXG|S_IXOTH);
        if(mkdir_res==-1)
        {
            cout<<"make dir failed!"<<endl;
            return false;
        }
        return true;
    }
    return true;

}

bool DataPreProcess::get_current_dir(string& current_dir)
{
	current_dir.clear() ;  
	char work_dir[260] = {0} ;  
	if(!getcwd(work_dir, 260))  
	{  
		return false ;  
	}  
  
	current_dir = work_dir;  
	current_dir.append("/");
  
	return true ;  
}

int main(int argc, char* argv[])
{
    if(argc==1)
    {
        cout<<"please inpute the bag file name..."<<endl;
        return -1;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("/ros_diff_controller/odom"));
    topics.push_back(std::string("/top_cam_img"));
    DataPreProcess dpp;
    bool extract_res = dpp.extract_file(argv[1],topics);
    if(extract_res)
        cout<<"finished"<<endl;
    bool syn_res = dpp.get_sychronise_data();
    dpp.select_data();

    return 0;
}
