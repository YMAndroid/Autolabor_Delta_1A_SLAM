/*********************************************************************************
File name:	  C3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_C3IROBOTICSLIDAR_H
#define EVEREST_LIDAR_C3IROBOTICSLIDAR_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket.h"
#include "CLidarUnpacket.h"
#include "CLidarPacketReceiver.h"
#include "CSerialConnection.h"
#include "CSimulateSerial.h"

/******************************* System libs includes *****************************/
#include <vector>

/******************************* Other libs includes ******************************/

namespace everest
{
	namespace hwdrivers
	{
	    struct TLidarScan
	    {
	        TLidarScan() : angle(), distance(), signal(){ }

	        void insert(TLidarScan &scan)
	        {
                this->angle.insert(this->angle.end(), scan.angle.begin(), scan.angle.end());
                this->distance.insert(this->distance.end(), scan.distance.begin(), scan.distance.end());
                this->signal.insert(this->signal.end(), scan.signal.begin(), scan.signal.end());
	        }

	        void clear()
	        {
	            angle.clear();
	            distance.clear();
	            signal.clear();
	        }

            size_t getSize() {return angle.size();}

	        std::vector<float> angle;
	        std::vector<float> distance;
	        std::vector<int>   signal;

	    };

        enum TLidarGrabResult
        {
            LIDAR_GRAB_ING = 0,
            LIDAR_GRAB_SUCESS,
            LIDAR_GRAB_ERRO,
            LIDAR_GRAB_ELSE
        };

        enum TLidarCommandID
        {
            I3LIDAR_DISTANCE = 0xA9,
            I3LIDAR_HEALTH   = 0xAB,
            I3LIDAR_LIDAR_SPEED  = 0xAE,
            I3LIDAR_NEW_DISTANCE = 0xAD
        };

		class C3iroboticsLidar
		{
            public:
                enum TGrabScanState
                {
                    GRAB_SCAN_FIRST = 0,
                    GRAB_SCAN_ELSE_DATA
                };

                /* Constructor */
                C3iroboticsLidar();

                /* Destructor */
                ~C3iroboticsLidar();

                /* Set device connect */
                bool initilize(CDeviceConnection *device_connect);

                /* Get scan data */
                TLidarGrabResult getScanData();

                /* Get Lidar Error */
                TLidarError getLidarError() { return m_lidar_erro; }

                /* Get lidar scan */
                TLidarScan& getLidarScan() { return m_lidar_scan; }

                /* Set data with signal*/
                void setDataWithSignal(bool data_with_signal) {m_data_with_signal = data_with_signal;}

                /* Return true if receive lidar speed */
                bool isReceiveLidarSpeed() {bool flag = m_receive_lidar_speed; m_receive_lidar_speed = false; return flag;}

                /* Get Lidar current speed */
                double getLidarCurrentSpeed() {return m_current_lidar_speed;}

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_receiver.enableLogWhenReceiveTimeOvers(state);}

            private:
                /* Analysis packet */
                TLidarGrabResult analysisPacket(CLidarPacket &lidar_packet);

                /* Analysis tooth scan */
                TLidarGrabResult analysisToothScan(CLidarPacket &lidar_packet);

                /* Analysis new tooth scan */
                TLidarGrabResult analysisNewToothScan(CLidarPacket &lidar_packet);

                /* Analysis health info */
                TLidarGrabResult analysisHealthInfo(CLidarPacket &lidar_packet);

                /* Analysis lidar speed */
                TLidarGrabResult analysisLidarSpeed(CLidarPacket &lidar_packet);

                /* Reset scan grab */
                void resetScanGrab();

                /* Combine scan */
                TLidarGrabResult combineScan(TToothScan &tooth_scan);

                /* Return true if tooth scan is first */
                bool isFirstScan(TToothScan &tooth_scan);

                /* Grab first scan */
                TLidarGrabResult grabFirstScan(TToothScan &tooth_scan);

                /* Change tooth scan to lidar scan */
                void toothScan2LidarScan(TToothScan &tooth_scan, TLidarScan &lidar_scan);

                /* Params */
                struct TParams
                {
                    /* Constructor */
                    TParams()
                    {
                        packet_wait_time_ms = 100;
                        tooth_number = 16;
                        scan_time_out_ms = 150;
                    }

                    size_t packet_wait_time_ms;
                    size_t scan_time_out_ms;
                    int tooth_number;
                };

            private:
                CDeviceConnection       *m_device_connect;
                CLidarPacketReceiver    m_receiver;
                TParams                 m_params;
                TLidarScan              m_lidar_scan;
                TLidarError             m_lidar_erro;
                TGrabScanState          m_grab_scan_state;
                int                     m_grab_scan_count;
                float                   m_last_scan_angle;
                CLidarPacket            m_packet;
                TToothScan              m_remainder_tooth_scan;
                bool                    m_remainder_flag;
                bool                    m_data_with_signal;

                bool                    m_receive_lidar_speed;
                double                  m_current_lidar_speed;

                CCountDown              m_data_count_down;
		};
	}
}

#endif


