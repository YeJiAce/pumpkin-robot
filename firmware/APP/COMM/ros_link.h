#ifndef PUMPKIN_ROS_LINK_H
#define PUMPKIN_ROS_LINK_H

#include "wire_codec.h"

void RosLink_Receive(const WireFrame *frame);
void RosLink_Poll(const WireParser *parser, uint32_t transport_drops);
void RosLink_TransportFault(void);

#endif
