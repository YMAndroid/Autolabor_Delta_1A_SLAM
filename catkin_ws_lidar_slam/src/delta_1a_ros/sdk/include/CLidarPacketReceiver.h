/*********************************************************************************
File name:	  CLidarPacketReceiver.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  lidar packet receiver
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/
#ifndef EVEREST_LIDAR_CLIDARPACKETRECEIVER_H
#define EVEREST_LIDAR_CLIDARPACKETRECEIVER_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket.h"

/******************************* Current libs includes ****************************/
#include "CCountDown.h"

/******************************* System libs includes *****************************/
#include <vector>
#include <fstream>

namespace everest
{
	namespace hwdrivers
	{
        class CDeviceConnection;

		class CLidarPacketReceiver
		{
            public:
                /* Constructor */
                CLidarPacketReceiver();

                /* Destructor */
                ~CLidarPacketReceiver();

                /* Receive lidar packet, if return true, it means receive a valid packet */
                bool receivePacket(CLidarPacket *packet);

				/* Sets the device this instance receives packets from */
				void setDeviceConnection(CDeviceConnection *device_connection) { m_device_conn = device_connection; }

				/* Gets the device this instance receives packets from */
				CDeviceConnection *getDeviceConnection(void)  { return m_device_conn; }

                enum TState
                {
                    STATE_HEADER1 = 0,
                    STATE_HEADER2,
                    STATE_LENGHT,
                    STATE_ACQUIRE_DATA
                };

                enum TPacketResult
                {
                    PACKET_ING = 0,
                    PACKET_SUCCESS,
                    PACKET_FAILED
                };

                /* Enable log when receive timer overs */
                void enableLogWhenReceiveTimeOvers(bool state) {m_log_when_receive_time_over = state;}

			private:
				/* Read packet, if it return ture, it means read complete packet */
				TPacketResult readPacket(CLidarPacket *packet, u8 ch);

				/* process state header1 */
				TPacketResult processStateHeader1(CLidarPacket *packet, u8 ch);

				/* process state header2 */
				TPacketResult processStateHeader2(CLidarPacket *packet, u8 ch);

				/* Process state length */
				TPacketResult processStateLength(CLidarPacket *packet, u8 ch);

				/* Process acuquire data */
				TPacketResult processStateAcquireData(CLidarPacket *packet, u8 ch);

                void reset();

            public:
                struct TParams
                {
                    /* Constructor */
                    TParams()
                    {
                        packet_max_time_ms = 1000;
                        packet_wait_time_ms = 100;
                    }

                    size_t packet_max_time_ms;
                    size_t packet_wait_time_ms;
                };

            private:
                CDeviceConnection 	*m_device_conn;
                CCountDown          m_count_down;
                TParams             m_params;
                TState              m_state;
                int                 m_actual_count;
                int                 m_packet_length;
                std::ofstream       m_save_fp;
                size_t              m_counter;
                bool                m_log_when_receive_time_over;
		};
	}
}

#endif


