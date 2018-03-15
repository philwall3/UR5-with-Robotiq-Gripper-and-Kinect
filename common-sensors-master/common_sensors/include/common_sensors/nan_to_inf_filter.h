/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */
#ifndef LASER_SCAN_NAN_TO_INF_FILTER_H
#define LASER_SCAN_NAN_TO_INF_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace sensor_filter {

class NanToInfFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
		
public:

	bool configure() {
		return true;
	}

	virtual ~NanToInfFilter(){}

	bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan){

		filtered_scan.ranges.resize(input_scan.ranges.size());

		float maxRangeSub=0.01; //descrease maximum range to filter out values that are just at the max range. This can be needed e.g. for gazebo which uses exact max range instead of infinity.

		for(unsigned int count = 0; count < input_scan.ranges.size(); ++count){
			filtered_scan.ranges[count] = input_scan.ranges[count];
		 
			if (isnan(input_scan.ranges[count]) || isinf(input_scan.ranges[count]) || 
			    (input_scan.ranges[count] < input_scan.range_min) || (input_scan.ranges[count] > (input_scan.range_max-maxRangeSub))) {
						
				filtered_scan.ranges[count] = std::numeric_limits<float>::infinity(); 
			}
		}

		//make sure to set all the needed fields on the filtered scan
		filtered_scan.header.frame_id = input_scan.header.frame_id;
		filtered_scan.header.stamp = input_scan.header.stamp;
		filtered_scan.angle_min = input_scan.angle_min;
		filtered_scan.angle_max = input_scan.angle_max;
		filtered_scan.angle_increment = input_scan.angle_increment;
		filtered_scan.time_increment = input_scan.time_increment;
		filtered_scan.scan_time = input_scan.scan_time;
		filtered_scan.range_min = input_scan.range_min;
		filtered_scan.range_max = input_scan.range_max;
		filtered_scan.intensities = input_scan.intensities;

		return true;

	}
};
}
#endif

