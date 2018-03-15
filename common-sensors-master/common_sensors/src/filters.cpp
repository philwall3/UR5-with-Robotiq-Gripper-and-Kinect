/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#include "common_sensors/footprint_filter.h"
#include "common_sensors/nan_to_inf_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(sensor_filter/FootprintFilter, sensor_filter::FootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_REGISTER_CLASS(sensor_filter/NanToInfFilter, sensor_filter::NanToInfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
//PLUGINLIB_REGISTER_CLASS(sensor_filter/NanRemoveFilter, sensor_filter::NanRemoveFilter, filters::FilterBase<sensor_msgs::LaserScan>)
