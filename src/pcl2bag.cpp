#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

using namespace boost::filesystem;

using namespace std;

vector<string> GetFileNames(boost::filesystem::path& dir, string ext);
bool FilesInDir(boost::filesystem::path& dir, string ext);
 
int main(int argc, char **argv) {

	ros::init(argc, argv, "pcl2bag");
	
	boost::filesystem::path dir("/home/gcielniak/Documents/20151117T220223_200");

	vector<string> file_names;

	cerr << dir << endl;

	//check if path is a folder
	if (boost::filesystem::is_directory(dir))	{
		file_names = GetFileNames(dir, ".pcd");
		cerr << file_names.size() << " pcd files found" << endl;
	} else {
		cerr << "No valid directory given!" << endl;
	}

	boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 100);
	ros::Publisher pub2 = n.advertise<sensor_msgs::Image>("image", 100);

	ros::Rate loop_rate(5);

	sensor_msgs::PointCloud2 cloud_msg;
	sensor_msgs::Image image_msg;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

//	rosbag::Bag bag;
//	bag.open("test.bag", rosbag::bagmode::Write);

	int i = 0;

	while (ros::ok() && (i < file_names.size())) {

		//extract timestamp
			boost::filesystem::path path(file_names[i]);
			boost::posix_time::ptime t = boost::posix_time::from_iso_string(path.filename().string().substr(6, 22));//image timestamp
			boost::posix_time::time_duration td(t - epoch);//time since epoch	
		//load file
			pcl::io::loadPCDFile<pcl::PointXYZRGBA> (file_names[i], cloud);
		//convert data type
			pcl::toROSMsg(cloud, cloud_msg);
			pcl::toROSMsg(cloud, image_msg);
			cloud_msg.header.frame_id = "base_link";
			cloud_msg.header.seq = i;
			cloud_msg.header.stamp = ros::Time::fromBoost(td);
			image_msg.header = cloud_msg.header;
		//write to bag
//			bag.write("pointcloud", ros::Time::fromBoost(td), cloud_msg);

		pub.publish(cloud_msg);
		pub2.publish(image_msg);

		ros::spinOnce();

		loop_rate.sleep();

		++i;
	}

//	bag.close();

	return 0;
}


bool FilesInDir(boost::filesystem::path& dir, string ext) {
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; ++pos)
		if (boost::filesystem::is_regular_file(pos->status()))
			if (boost::filesystem::extension(*pos) == ext)
				return true;

	return false;
}

vector<string> GetFileNames(boost::filesystem::path& dir, string ext) {
	vector<string> files;
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; ++pos) {
		if (boost::filesystem::is_regular_file(pos->status())) {
			if (boost::filesystem::extension(*pos) == ext) {
			#if BOOST_FILESYSTEM_VERSION == 3
				files.push_back(pos->path().string());
			#else
				files.push_back(pos->path());
			#endif
			}
		}
	}

	return files;
}
