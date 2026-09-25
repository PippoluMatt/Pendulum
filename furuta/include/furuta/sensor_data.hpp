#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP


namespace furuta {

struct SensorData
{
    double theta{0.0};       // rad
    double alpha{0.0};       // rad
    
    double thetaDotFiltered{0.0};    // rad/s
    double alphaDotFiltered{0.0};    // rad/s
};

} // furuta namespace
#endif 