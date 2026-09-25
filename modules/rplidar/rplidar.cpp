#include <pond/pond.hpp>
#include "sl_lidar.h"
#include <pond_data_types/laser_scan_types.hpp>

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID   = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID   = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID   = 8,
};

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RESET_TIMEOUT 15 // 15 second

class RPLidar : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    pond::Distributor distributor;

    void distribute_scan(
        sl_lidar_response_measurement_node_hq_t *nodes,
        size_t node_count, double start,
        double scan_time,
        float angle_min, float angle_max
    )
    {
        LaserScanSPtr scan = std::make_shared<LaserScan>();

        scan->stamp.time = start;
        scan->stamp.hw_time = start;
        scan->stamp.frame_id = frame_id;
        
        bool reversed = (angle_max > angle_min);

        if (reversed)
        {
            scan->angle_min =  M_PI - angle_max;
            scan->angle_max =  M_PI - angle_min;
        }
        else
        {
            scan->angle_min =  M_PI - angle_min;
            scan->angle_max =  M_PI - angle_max;
        }
        
        scan->angle_increment = (scan->angle_max - scan->angle_min) / (double)(node_count-1);
        scan->scan_time = scan_time;
        scan->time_increment = scan_time / (double)(node_count-1);
        scan->range_min = 0.15;
        scan->range_max = max_distance;

        scan->intensities.resize(node_count);
        scan->ranges.resize(node_count);

        size_t scan_midpoint = node_count / 2;

        for (size_t i = 0; i < node_count; i++)
        {
            size_t apply_index = (inverted != reversed ? node_count - 1 - i : i);
            if (flip_x_axis) apply_index += (apply_index >= scan_midpoint ? -scan_midpoint : scan_midpoint);
            
            scan->ranges[apply_index] = (nodes[i].dist_mm_q2 ? (float)nodes[i].dist_mm_q2 / 4.0f / 1000 : std::numeric_limits<float>::infinity());
            scan->intensities[apply_index] = (float)(nodes[apply_index].quality >> 2);
        }

        distributor.distribute(&scan);
    }

    bool checkRPLIDARHealth(sl::ILidarDriver* drv)
    {
        sl_lidar_response_device_health_t healthinfo;
        sl_result op_result = drv->getHealth(healthinfo);

        if (SL_IS_OK(op_result))
        { 
            POND_LOG("RPLidar health status : %d", healthinfo.status);
            switch (healthinfo.status)
            {
            case SL_LIDAR_STATUS_OK:
                POND_LOG("RPLidar health status : OK.");
                return true;
            case SL_LIDAR_STATUS_WARNING:
                POND_LOG("RPLidar health status : Warning.");
                return true;
            case SL_LIDAR_STATUS_ERROR:
                POND_LOG("Error: RPLidar internal error detected. Please reboot the device to retry.");
                return false;
            default:
                POND_LOG("Error: Unknown internal error detected. Please reboot the device to retry.");
                return false;
            }
        }

        POND_LOG("Error: cannot retrieve RPLidar health code: %x", op_result);
        return false;
    }

    bool set_scan_mode()
    {
        sl_result op_result;
        sl::LidarScanMode current_scan_mode;

        if (scan_mode.empty()) op_result = drv->startScan(false, true, 0, &current_scan_mode);
        else
        {
            std::vector<sl::LidarScanMode> allSupportedScanModes;
            op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

            if (SL_IS_OK(op_result))
            {
                sl_u16 selectedScanMode = sl_u16(-1);
                for (std::vector<sl::LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
                {
                    if (iter->scan_mode == scan_mode)
                    {
                        selectedScanMode = iter->id;
                        break;
                    }
                }

                if (selectedScanMode == sl_u16(-1))
                {
                    POND_LOG("Error: scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<sl::LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
                        POND_LOG("Error: \t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode, iter->max_distance, (1000 / iter->us_per_sample));
                    
                    op_result = SL_RESULT_OPERATION_FAIL;
                }
                else op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }

        if (SL_IS_OK(op_result))
        {
            //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
            int points_per_circle = (int)(1000 * 1000 / current_scan_mode.us_per_sample / scan_frequency);

            if ((angle_compensate_multiple = points_per_circle / 360.0 + 1) < 1) angle_compensate_multiple = 1.0;
            max_distance = (float)current_scan_mode.max_distance;

            POND_LOG("current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", current_scan_mode.scan_mode, (int)(1000 / current_scan_mode.us_per_sample + 0.5), max_distance, scan_frequency);
            return true;
        }
        else
        {
            POND_LOG("Error: Can not start scan: %08x!", op_result);
            return false;
        }
    }

    float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    bool start()
    {
        POND_LOG("Start");
        drv->setMotorSpeed();
        if (!set_scan_mode())
        {
            stop();
            POND_LOG("Failed to set scan mode");
            return false;
        }
        is_scanning = true;
        return true;
    }

    void stop()
    {
        POND_LOG("Stop");
        drv->stop();
        drv->setMotorSpeed(0);
        is_scanning = false;
    }

    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    bool flip_x_axis = false;
    bool auto_standby = false;
    double max_distance = 8.0;
    size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    double scan_frequency;
    bool is_scanning = false;

    sl::ILidarDriver *drv = nullptr;
    bool scan_frequency_tunning_after_scan = false;
};

POND_MODULE_CPP_DECLARE(RPLidar, "lidar", "modulefor rplidars")

POND_BUNDLE_DECLARE(
    "rplidar bundle",
    POND_MODULE(RPLidar),
)

pond_result RPLidar::onStartup(const std::vector<void*>& args)
{
    std::string channel_type, tcp_ip, udp_ip, serial_port;
    int32_t tcp_port, udp_port, serial_baudrate;

    if (auto opt = parameter("channel_type").asString().getStrict({"tcp", "udp", "serial"})) channel_type = *opt; else return POND_ERROR;

    tcp_ip = parameter("tcp_ip").asString().get("192.168.0.7"); 
    tcp_port = parameter("tcp_port").asInt().get(20108);
    udp_ip = parameter("udp_ip").asString().get("192.168.11.2"); 
    udp_port = parameter("udp_port").asInt().get(8089);

    if (channel_type == "serial")
    {
        if (auto opt = parameter("serial_port").asString().getStrict()) serial_port = *opt; else return POND_ERROR;
        if (auto opt = parameter("serial_baudrate").asInt().getStrict()) serial_baudrate = *opt; else return POND_ERROR;
    }

    if (auto opt = parameter("frame_id").asString().getStrict()) frame_id = *opt; else return POND_ERROR;

    inverted = parameter("inverted").asBool().get(false);
    angle_compensate = parameter("angle_compensate").asBool().get(false);
    flip_x_axis = parameter("flip_x_axis").asBool().get(false);
    auto_standby = parameter("auto_standby").asBool().get(false);
    scan_mode = parameter("scan_mode").asString().get("");
    scan_frequency = parameter("scan_frequency").asDouble().get(channel_type == "udp" ? 20.0 : 10.0);

    POND_LOG("RPLIDAR SDK Version:%d.%d.%d", SL_LIDAR_SDK_VERSION_MAJOR, SL_LIDAR_SDK_VERSION_MINOR, SL_LIDAR_SDK_VERSION_PATCH);    

    if ((drv = *sl::createLidarDriver()) == nullptr)
    {   /* don't start spinning without a driver object */
        POND_LOG("Error: Failed to construct driver");
        return POND_ERROR;
    }

    sl::IChannel* _channel;
    if(channel_type == "tcp")       _channel = *sl::createTcpChannel(tcp_ip, tcp_port);
    else if(channel_type == "udp")  _channel = *sl::createUdpChannel(udp_ip, udp_port);
    else                            _channel = *sl::createSerialPortChannel(serial_port, serial_baudrate);
    
    if (SL_IS_FAIL((drv)->connect(_channel)))
    {
        if(channel_type == "tcp")
            POND_LOG("Error: cannot connect to the ip addr  %s with the tcp port %d", tcp_ip.c_str(), tcp_port);
        
        else if(channel_type == "udp")
            POND_LOG("Error: cannot connect to the ip addr  %s with the udp port %d", udp_ip.c_str(), udp_port);
        
        else POND_LOG("Error: cannot bind to the specified serial port %s.", serial_port.c_str());            
    
        delete drv; return POND_ERROR;
    }
    
    sl_lidar_response_device_info_t devinfo;
    sl_result op_result = drv->getDeviceInfo(devinfo);

    if (SL_IS_FAIL(op_result))
    {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) POND_LOG("Error: operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
        else POND_LOG("Error: unexpected error, code: %x",op_result);

        delete drv; return POND_ERROR;
    }

    // print out the device serial number, firmware and hardware version number..
    char sn_str[37] = {'\0'}; 
    for (int pos = 0; pos < 16 ;++pos) sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
    
    POND_LOG("RPLidar S/N: %s", sn_str);
    POND_LOG("Firmware Ver: %d.%02d", devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    POND_LOG("Hardware Rev: %d", (int)devinfo.hardware_version);
    
    if (!checkRPLIDARHealth(drv)) { delete drv; return POND_ERROR; }

    if ((devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID) scan_frequency_tunning_after_scan = true;

    //for RPLIDAR A serials; start RPLIDAR A serials  rotate by pwm
    if(!scan_frequency_tunning_after_scan) drv->setMotorSpeed(600);

    if (!auto_standby && !start()) {delete drv; return POND_ERROR;}

    distributor = createDistributor<LaserScanSPtr>({"scan"});

    return POND_SUCCESS;
}

void RPLidar::onShutdown()
{
    distributor.destroy();

    drv->setMotorSpeed(0);
    drv->stop();
    POND_LOG("Stop motor");
    delete drv;
}

void RPLidar::onFrame()
{
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;

    double start_scan_time = pond::get_time();
    sl_result op_result = drv->grabScanDataHq(nodes, count);
    double end_scan_time = pond::get_time();
    double scan_duration = end_scan_time - start_scan_time;

    if (op_result == SL_RESULT_OK)
    {
        if(scan_frequency_tunning_after_scan)
        { //Set scan frequency(For Slamtec Tof lidar)
            POND_LOG( "set lidar scan frequency to %.1f Hz(%.1f Rpm) ", scan_frequency, scan_frequency*60);
            drv->setMotorSpeed(scan_frequency*60); //rpm 
            scan_frequency_tunning_after_scan = false;
            return;
        }

        op_result = drv->ascendScanData(nodes, count);

        if (op_result == SL_RESULT_OK)
        {
            if (angle_compensate)
            {
                const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                int angle_compensate_offset = 0;
                auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                size_t i = 0, j = 0;
                for( ; i < count; i++ )
                {
                    if (nodes[i].dist_mm_q2 != 0)
                    {
                        float angle = getAngle(nodes[i]);

                        int angle_value = (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        
                        for (j = 0; j < angle_compensate_multiple; j++)
                        {
                            int angle_compensate_nodes_index = angle_value-angle_compensate_offset + j;
                            if(angle_compensate_nodes_index >= angle_compensate_nodes_count) angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                            angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                        }
                    }
                }

                distribute_scan(
                    angle_compensate_nodes, angle_compensate_nodes_count,
                    start_scan_time, scan_duration,
                    DEG2RAD(0.0f), DEG2RAD(359.0f)
                );

                if (angle_compensate_nodes) delete[] angle_compensate_nodes;
            }
            else
            {
                int start_node = 0, end_node = 0, i = 0;
                // find the first valid node and last valid node
                while (nodes[i++].dist_mm_q2 == 0);
                start_node = i-1;
                i = count -1;
                while (nodes[i--].dist_mm_q2 == 0);
                end_node = i+1;

                distribute_scan(
                    &nodes[start_node], end_node-start_node +1,
                    start_scan_time, scan_duration, 
                    DEG2RAD(getAngle(nodes[start_node])), DEG2RAD(getAngle(nodes[end_node]))
                );
            }
        }
        // All the data is invalid, just publish them
        else if (op_result == SL_RESULT_OPERATION_FAIL) distribute_scan(
            nodes, count,
            start_scan_time, scan_duration,
            DEG2RAD(0.0f), DEG2RAD(359.0f)
        );
    }
}