#include "Openni.h"
#include <pcl/io/openni_grabber.h>
#include <boost/shared_array.hpp>

typedef union
{
	struct /*anonymous*/
	{
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	float float_value;
	long long_value;
} RGBValue;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr SimpleOpenNIViewer::convertToXYZRGBPointCloud(const boost::shared_ptr<openni_wrapper::Image> &image,
																				const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image, float constant									
																				) 
{
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());

	cloud->header.frame_id = "/openni_rgb_optical_frame";// rgb_frame_id_;

	// just guess
	int depth_width_ = 640;
	int depth_height_ = 480;
	int image_width_ = 640;
	int image_height_ = 480;

	cloud->height = depth_height_;
	cloud->width = depth_width_;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);

	register const XnDepthPixel* depth_map = depth_image->getDepthMetaData().Data();
	if (depth_image->getWidth() != depth_width_ || depth_image->getHeight() != depth_height_)
	{
		static unsigned buffer_size = 0;
		static boost::shared_array<unsigned short> depth_buffer(0);

		if (buffer_size < depth_width_ * depth_height_)
		{
			buffer_size = depth_width_ * depth_height_;
			depth_buffer.reset(new unsigned short [buffer_size]);
		}

		depth_image->fillDepthImageRaw(depth_width_, depth_height_, depth_buffer.get());
		depth_map = depth_buffer.get();
	}

	// here we need exact the size of the point cloud for a one-one correspondence! 
	if (rgb_array_size < image_width_ * image_height_ * 3)
	{
		rgb_array_size = image_width_ * image_height_ * 3;
		rgb_array.reset(new unsigned char [rgb_array_size]);
		rgb_buffer = rgb_array.get();
	}
	image->fillRGB(image_width_, image_height_, rgb_buffer, image_width_ * 3);

	// depth_image already has the desired dimensions, but rgb_msg may be higher res.
	register int color_idx = 0, depth_idx = 0;
	RGBValue color;
	color.Alpha = 0;

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0 ||
				depth_map[depth_idx] == depth_image->getNoSampleValue() ||
				depth_map[depth_idx] == depth_image->getShadowValue())
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z = depth_map[depth_idx] * 0.001f;
				pt.x = u * pt.z * constant;
				pt.y = v * pt.z * constant;
			}

			// Fill in color
			color.Red = rgb_buffer[color_idx];
			color.Green = rgb_buffer[color_idx + 1];
			color.Blue = rgb_buffer[color_idx + 2];
			pt.rgb = color.float_value;
		}
	}
	return (cloud);
}
