/**
 * @file dynamicParameters.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief 
 * @version 0.1
 * @date 2019-06-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */
 
#include <ros/ros.h>
#include <data_treatment/filtersConfig.h>
#include <dynamic_reconfigure/server.h>

void callback(data_treatment::filtersConfig &config, uint32_t level)
{
 
}


  int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamicParameters");

    dynamic_reconfigure::Server<data_treatment::filtersConfig> server;
    dynamic_reconfigure::Server<data_treatment::filtersConfig>::CallbackType f;

    f=boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    
    return 0;
}