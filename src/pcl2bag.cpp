#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

using namespace boost::filesystem;

using namespace std;

vector<string> GetFileNames(boost::filesystem::path& dir, string ext);
bool FilesInDir(boost::filesystem::path& dir, string ext);

   /** \brief Copy the RGB fields of a PointCloud into pcl::PCLImage format
     * \param[in] cloud the point cloud message
     * \param[out] msg the resultant pcl::PCLImage
     * CloudT cloud type, CloudT should be akin to pcl::PointCloud<pcl::PointXYZRGBA>
     * \note will throw std::runtime_error if there is a problem
     */

template<typename CloudT> void
toPCLDepthPointCloud2 (CloudT& cloud, pcl::PCLImage& msg)
{
 // Ease the user's burden on specifying width/height for unorganized datasets
 if (cloud.width == 0 && cloud.height == 0)
   throw std::runtime_error("Needs to be a dense like cloud!!");
 else
 {
   if (cloud.points.size () != cloud.width * cloud.height)
     throw std::runtime_error("The width and height do not match the cloud size!");
   msg.height = cloud.height;
   msg.width = cloud.width;
 }

 msg.encoding = "16UC1";
 msg.step = msg.width * sizeof (uint16_t);
 msg.data.resize (msg.step * msg.height);
 for (size_t y = 0; y < cloud.height; y++)
 {
   for (size_t x = 0; x < cloud.width; x++)
   {
     uint16_t* pixel = (uint16_t*)&(msg.data[y * msg.step + x*sizeof(uint16_t)]);
		*pixel = cloud (x, y).z*1000;
   }
 }
}
 
int main(int argc, char **argv) {

	ros::init(argc, argv, "pcl2bag");
	
	boost::filesystem::path dir("/media/data/data/broccoli/20151104T130053_data2_pcd");

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
	ros::Publisher pub2 = n.advertise<sensor_msgs::Image>("camera/rgb/image_rect_color", 100);
	ros::Publisher pub3 = n.advertise<sensor_msgs::Image>("camera/depth_registered/image_raw", 100);
	ros::Publisher pub4 = n.advertise<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 100);

	ros::Rate loop_rate(5);

	sensor_msgs::PointCloud2 cloud_msg;
	sensor_msgs::Image image_msg;
	sensor_msgs::Image depth_image_msg;
	sensor_msgs::CameraInfo camera_info;
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
			pcl::PCLImage pcl_image;
			toPCLDepthPointCloud2< pcl::PointCloud<pcl::PointXYZRGBA> >(cloud, pcl_image);
			pcl_conversions::moveFromPCL(pcl_image, depth_image_msg);
			cloud_msg.header.frame_id = "base_link";
			cloud_msg.header.seq = i;
//			cloud_msg.header.stamp = ros::Time::fromBoost(td);
			cloud_msg.header.stamp = ros::Time::now();//fromBoost(td);

			image_msg.header = cloud_msg.header;
			image_msg.header.frame_id = "camera_rgb_optical_frame";
			depth_image_msg.header = image_msg.header;

			camera_info.header = image_msg.header;
			camera_info.header.frame_id = "camera_rgb_optical_frame";
			camera_info.distortion_model = "plum_bob";
			camera_info.height = cloud_msg.height;
			camera_info.width = cloud_msg.width;
			camera_info.D.resize(5);
			camera_info.K[0] = camera_info.K[4] = 364.82281494140625; //fx=fy
			camera_info.K[2] = 255.5; //cx
			camera_info.K[5] = 211.5; //cy
			camera_info.K[8] = 1;
			camera_info.P[0] = camera_info.P[5] = 364.82281494140625; //fx=fy
			camera_info.P[2] = 255.5; //cx
			camera_info.P[6] = 211.5; //cy
			camera_info.P[10] = 1;
			camera_info.R[0] = camera_info.R[4] = camera_info.R[8] = 1;
 		
		//write to bag
//			bag.write("pointcloud", ros::Time::fromBoost(td), cloud_msg);

		pub.publish(cloud_msg);
		pub2.publish(image_msg);
		pub3.publish(depth_image_msg);
		pub4.publish(camera_info);

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
