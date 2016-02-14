/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/mobility-module.h"

#include <iostream>
#include <string>
#include <cmath>
#include <cassert>
#include <ctime>
#include <sstream>
#include <time.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/discrete_distribution.hpp>
#include <boost/timer.hpp>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FinalModel");


/*======== Scenario ===*/
int numVehicles;
int numOfPackets;
double interval;

/*======== The global variables ===*/
uint32_t RandomDistSequence[1000][1000]; // the number shows the max amount of cars(left) and Max packet(right).
uint32_t channelAssign[1000][1000];		 // channel assignment for a packet
uint8_t port = 80;					   	 // For UDP CAM application.
uint32_t tempPktCount[1000];			 // Guard for the transmit payload. [number of vehicle(vehicleID)]
std::string sendPayload[1000];			 // Guard for the transmit payload. [number of vehicle(vehicleID)]
Time packeActivationtTime[1000];		 // Time when a message is created.
Vector SenderPosistion[1000];	 	     // the number shows the max amount of car.(represents car ID).
int TotalMessageTxRx[1000][3];		     // the left number represents car ID and the left number 0=CCH, 1=SCH1, 2=SCH2.
int TotalMessageTx[1000][3];		     // the left number represents car ID and the left number 0=CCH, 1=SCH1, 2=SCH2.
Time Chronos = Seconds (13.0);		     // used as Max time for the CBR calculation and statistic collection.
Time MsgDuration = MilliSeconds(0.0);    // duration for a transceiver to send/receive a particular size of message.
double dMaxMessagePerDuration = 0.0;     // Max number of messages in a period of CBRprobingInterval.
Time CBRprobingInterval;				 // TS 102 636-4-2 and Tiels et al. triggers the calculation of Global CBR.
double dGlobalCBR[1000][3];		  		 // Global CBR value per channel.
double dLocalCBR[1000][3];				 // local CBR value, the left number represents car ID and the right number 0=CCH, 1=SCH1, 2=SCH2.
bool isPhyBusy[1000][3];		     	 // Transceiver status
int ExtLocTable1hop[1000][1001];  		 // <<== [vehID][NumOfVehicles(content)]
int ExtLocTable2hop[1000][1001];  		 // <<== [vehID][NumOfVehicles(content)]
int tempExtLocTable[1000][1001];
/*=================================*/

/*======== CSMA/CA broadcast variables ========*/
uint32_t aifs[1000][1000][3];			// AIFS time slot [vecID][pktSequence][Channel]
int bf[1000][1000][3];					// back off = 7 slot [vecID][pktSequence][Channel]
bool do_bf[1000][1000][3];				// boolean value to check the back off counting.
int cwMax = 1023;						// Contention window ETSI TS 102 636-4-2 V1.1.1
int cwMin = 15;							// Contention window ETSI TS 102 636-4-2 V1.1.1
uint32_t Aifs = 11;						// AIFS slot ETSI TS 102 636-4-2 V1.1.1
Time timeSLot = MicroSeconds(13.0);
Time aifsTimeSLot = MicroSeconds(10.0);
Time packetJitter[1000];				// packet
/*=================================*/

/*======== Statistic variables === */
int ReceivePktCounter[1000][1000];  // the number shows the max amount of car.(left, represents receiver car ID. right sender ID). <====== WARNING!! Don't Forget!!
uint32_t DropPktCounter[1000][3];     	 // MD per vehicle per channel. [Vehicle ID][Channel]
Time InfoAge[1000];						 // the number shows the max amount of car.(left, represents receiver car ID. right sender ID). <====== WARNING!! Don't Forget!!
Time LastInfoUpdate[1000][1000];
Time Age[1000][1000];
double SMR[1000];						 // SMR per vehicle.
/*=================================*/

// Additional Function to
std::string PayloadToString(int iCarID, int size_of_vehicle) {
  std::ostringstream oss("");
  oss << Simulator::Now() << ",";
  oss << iCarID << ",";
  for (int temp = 0; temp < size_of_vehicle; temp++)
    oss << ExtLocTable1hop[iCarID][temp] << ",";
  return oss.str();
}

std::string ExtractTimeStamp(std::string data) {
  std::ostringstream oss("");
  uint32_t i=0;
  do{
	  oss << data[i];
	  i++;
  }while(data[i]!=',');
  return oss.str();
}

void statisticData()
{
	std::cout<<"# Time(ms), all SMR, MD-CCH, MD-SCH1, MD-SCH2, all vec IA (ms), " << std::endl;
	int Domain1st=0, Domain2nd=0, Domain3rd=0;
	for (int i = 0; i<numVehicles; i++){
		if (i%3==0){Domain1st=Domain1st+1;}
		if (i%3==1){Domain2nd=Domain2nd+1;}
		if (i%3==2){Domain3rd=Domain3rd+1;}
	}

	if (Simulator::Now() <= Chronos){
		Time dummy = Simulator::Now();
		double time = (dummy.GetDouble()/1000000);
		std::cout<<time<<", ";

		//==SMR==
		double strategySMR;
		for (int i=0; i<numVehicles; i++){
			if(i%3==0){ 		// 1st domian vehicle received all messages from all vehicles
				for(int j=0;j<numVehicles; j++){
					if (i!=j){
						SMR[i] = SMR[i] + ReceivePktCounter[i][j];
					}
				}
				strategySMR = strategySMR + SMR[i]/((numVehicles-1)*numOfPackets);
			} else if(i%3==1){ // 2nd domian vehicle received messages from vehicles in the 1st and 2nd domain
				for(int j=0;j<numVehicles; j+=3){
					if (i!=j){
						SMR[i] = SMR[i] + ReceivePktCounter[i][j];
					}
				}
				for(int j=1;j<numVehicles; j+=3){
					if (i!=j){
						SMR[i] = SMR[i] + ReceivePktCounter[i][j];
					}
				}
				strategySMR = strategySMR + SMR[i]/(((Domain2nd+Domain1st)-1)*numOfPackets);
			} else if(i%3==2){ // 3rd domain vehicle received messages from vehicles in the 1st and 3rd domain
				for(int j=0;j<numVehicles; j+=3){
					if (i!=j){
						SMR[i] = SMR[i] + ReceivePktCounter[i][j];
					}
				}
				for(int j=2;j<numVehicles; j+=3){
					if (i!=j){
						SMR[i] = SMR[i] + ReceivePktCounter[i][j];
					}
				}
				strategySMR = strategySMR + SMR[i]/(((Domain3rd+Domain1st)-1)*numOfPackets);
			}
			//std::cout << SMR[i]/((numVehicles-1)*numOfPackets) << ", ";
		}
		std::cout<<(strategySMR/numVehicles)*100<<", ";
		for (int i=0; i<numVehicles; i++){
	  		SMR[i] = 0.0;
	  	}

		//==MD==
		double avgDropCCH=0, avgDropSCH1=0, avgDropSCH2=0;
		for (int i=0; i<numVehicles; i++){
			avgDropCCH = avgDropCCH + DropPktCounter[i][0];
			avgDropSCH1 = avgDropSCH1 + DropPktCounter[i][1];
			avgDropSCH2 = avgDropSCH2 + DropPktCounter[i][2];
			//std::cout<<Simulator::Now()/1000000<<", "<<DropPktCounter[i][0]<<", "<<DropPktCounter[i][1]<<", "<< DropPktCounter[i][2]<<std::endl;
	  	}
		std::cout<<(avgDropCCH/(numVehicles*numOfPackets))*100<<", "
				<<(avgDropSCH1/(numVehicles*numOfPackets))*100<<", "
				<<(avgDropSCH2/(numVehicles*numOfPackets))*100<<", ";

		  // ============ Information Age ========================
		  Time InformationAge;
		  for(int i=0;i<numVehicles;i++){
			  if(i%3==0){
				  for(int j=0;j<numVehicles;j++){
					  if(i!=j&&ReceivePktCounter[i][j]!=0){InfoAge[i]+= Age[i][j]/ReceivePktCounter[i][j];}
				  }
				  InformationAge+= InfoAge[i]/(numVehicles-1);

			  } else if(i%3==1){
				  for(int j=0;j<numVehicles;j+=3){
				  			  if(i!=j&&ReceivePktCounter[i][j]!=0){InfoAge[i]+= Age[i][j]/ReceivePktCounter[i][j];}
				  		  }
				  for(int j=1;j<numVehicles;j+=3){
				  			  if(i!=j&&ReceivePktCounter[i][j]!=0){InfoAge[i]+= Age[i][j]/ReceivePktCounter[i][j];}
				  		  }
				  InformationAge+= InfoAge[i]/((Domain1st+Domain2nd)-1);

			  } else if(i%3==2){
				  for(int j=0;j<numVehicles;j+=3){
				  			  if(i!=j&&ReceivePktCounter[i][j]!=0){InfoAge[i]+= Age[i][j]/ReceivePktCounter[i][j];}
				  		  }
				  for(int j=2;j<numVehicles;j+=3){
				  			  if(i!=j&&ReceivePktCounter[i][j]!=0){InfoAge[i]+= Age[i][j]/ReceivePktCounter[i][j];}
				  		  }
				  InformationAge+= InfoAge[i]/((Domain1st+Domain3rd)-1);
			  }
		  }

		  std::cout<<InformationAge.GetMilliSeconds()/numVehicles<<std::endl;


		  double PhyCCHload=0,PhySCH1load=0,PhySCH2load=0;
		  for (int i=0;i<numVehicles;i++){
		    PhyCCHload=PhyCCHload+TotalMessageTx[i][0];
		    PhySCH1load=PhySCH1load+TotalMessageTx[i][1];
		    PhySCH2load=PhySCH2load+TotalMessageTx[i][2];
		  }

		  std::cout <<"# Load Distribution"<<std::endl;
		  std::cout	<< (PhyCCHload/(numOfPackets*numVehicles))*100 <<","
		    << (PhySCH1load/(numOfPackets*numVehicles))*100 <<","
		    << (PhySCH2load/(numOfPackets*numVehicles))*100 <<"" << std::endl;

		  std::cout<<"# SMR per Vehicle"<<std::endl;
		  std::cout<<"# VehicleID,% loss #"<<std::endl;

		  for (int i=0; i<numVehicles; i++){
			  if(i%3==0){ 		// 1st domain vehicle received all messages from all vehicles
				  for(int j=0;j<numVehicles; j++){
					  if (i!=j){SMR[i] = SMR[i] + ReceivePktCounter[j][i];}
					}
					std::cout << i <<", "<< 100-((SMR[i]/((numVehicles-1)*numOfPackets))*100) << std::endl;
			  } else if(i%3==1){ // 2nd domain vehicle received messages from vehicles in the 1st and 2nd domain
				  for(int j=0;j<numVehicles; j+=3){
					  if (i!=j){ SMR[i] = SMR[i] + ReceivePktCounter[j][i];}
					}
				  for(int j=1;j<numVehicles; j+=3){
					  if (i!=j){SMR[i] = SMR[i] + ReceivePktCounter[j][i];}
					}
					std::cout << i <<", "<< 100-((SMR[i]/((Domain2nd+Domain1st-1)*numOfPackets))*100) << std::endl;
			  } else if(i%3==2){ // 3rd domain vehicle received messages from vehicles in the 1st and 3rd domain
				  for(int j=0;j<numVehicles; j+=3){
					  if (i!=j){SMR[i] = SMR[i] + ReceivePktCounter[j][i];}
				  }
				  for(int j=2;j<numVehicles; j+=3){
					  if (i!=j){SMR[i] = SMR[i] + ReceivePktCounter[j][i];}
				  }
				  std::cout << i <<", "<< 100-((SMR[i]/((Domain3rd+Domain1st-1)*numOfPackets))*100) << std::endl;
			  }
		  }

	}
}

// Transmit Receive Function
double phasing(uint32_t car){
	boost::random::mt19937 gen;
	int seeder = (int)time(NULL)+(car+1);
	gen.seed(seeder);
	boost::random::uniform_int_distribution<> dist(0, 98000000);
	return dist(gen);
}

double jitter(uint32_t car, uint32_t packet){
	boost::random::mt19937 gen;
	int seeder = (int)time(NULL)+((car+1)*(packet+1));
	gen.seed(seeder);
	boost::random::uniform_int_distribution<> dist(0, 2);
	return dist(gen);
}

void GenerateBF(uint32_t car, uint32_t packet, uint32_t channel){
	boost::random::mt19937 gen;
	int seeder = (int)time(NULL)+((car+1)*packet*(channel+1));
	gen.seed(seeder);
	boost::random::uniform_int_distribution<> dist(cwMin, cwMax);
	bf[car][packet][channel]=dist(gen);
}

void randomGenerator(uint32_t roll, uint32_t car) { // create equal distribution message to all channel.
	boost::random::mt19937 gen;
	int seeder = (int)time(NULL)+car;
	gen.seed(seeder);
    boost::random::uniform_int_distribution<> dist(1, 3);  	// generate the random numbers for 1, 2, and 3.
    float counter1 = ceil((float)roll/3); 					// counter to distribute the packet randomly equal
    float counter2 = ceil((float)roll/3);
    float counter3 = ceil((float)roll/3);
    uint32_t i = roll;
    uint32_t h = car-1;
   		do{
			uint32_t temp = dist(gen);
			//std::cout << "nilai h:" << h << ";" << counter1 << ";" << counter2<< ";" << counter3<< std::endl;
			if (temp==1 && counter1>0){
				RandomDistSequence[h][i] = temp;
				counter1--;
				i--;
			} else if (temp==2 && counter2>0){
				RandomDistSequence[h][i] = temp;
				counter2--;
				i--;
			} else if (temp==3 && counter3>0){
				RandomDistSequence[h][i] = temp;
				counter3--;
				i--;
			}
		}while(i > 0);
}

void ReceivePacket (Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	Address from;
	uint32_t iReceiverID = socket->GetNode()->GetId();
	Time pktInterval = MilliSeconds(interval);

	while (packet = socket->RecvFrom(from))
    {
		// ==== Extract Payload (i.e. TimeStamp and sender LocT) from the incoming packet and merge with the LocTable.==============
		uint8_t *buffer = new uint8_t[packet->GetSize()];
		packet->CopyData (buffer, packet->GetSize());
		std::string Payload = std::string((char*)buffer);
		std::string timeinfo = ExtractTimeStamp(Payload);

		boost::erase_head(Payload,timeinfo.length()+1);
		boost::replace_first(timeinfo, "+", " "); boost::replace_first(timeinfo, "ns", " "); boost::erase_all(timeinfo, " ");
		double dummy = boost::lexical_cast<double>(timeinfo);
		Time pktTimeStamp = NanoSeconds(dummy);

		//std::cout<< pktTimeStamp<< " <--Time stamp   :   Information age :" <</*(pktTimeStamp/2)*/ (Simulator::Now()-pktTimeStamp) << std::endl;
		//std::cout<< Payload << std::endl;

		boost::replace_all(Payload,","," ");
		int i = 0;
		std::stringstream ssin(Payload);
		while (ssin.good() && i < numVehicles+1){     // extra one payload i.e. senderID
		     ssin >> tempExtLocTable[iReceiverID][i];
		     i++;
		}
		//for(int j = 0; j < numVehicles; j++){std::cout << iReceiverID <<":"<< tempExtLocTable[iReceiverID][j] << " temp ext table "<<std::endl;}

		/* =================== Not used in Random =====================================

		// ===== Merge incoming message 0 hop info and covert as 1 hop info ==========
		int ext1hopCoutner = 0;
		do{
			if (tempExtLocTable[iReceiverID][0]==ExtLocTable1hop[iReceiverID][ext1hopCoutner])
			{break;}
			else if (ExtLocTable1hop[iReceiverID][ext1hopCoutner]==-1)
			{ExtLocTable1hop[iReceiverID][ext1hopCoutner]=tempExtLocTable[iReceiverID][0];break;}
			ext1hopCoutner++;
		}while(true);

		//for(int j = 0; j < numVehicles; j++){std::cout << iReceiverID<<":"<< ExtLocTable1hop[iReceiverID][j] << " <-- 1 hop info"<< std::endl;}

		// ===== Merge incoming message 1 hop info and covert as 2 hop info ==========
		int tempCounter=1;
		do{
			int extCounter=0;
			do{
				if(ExtLocTable2hop[iReceiverID][extCounter]!=tempExtLocTable[iReceiverID][tempCounter] && ExtLocTable2hop[iReceiverID][extCounter]==-1){
					ExtLocTable2hop[iReceiverID][extCounter]=tempExtLocTable[iReceiverID][tempCounter]; break;
				} else if (ExtLocTable2hop[iReceiverID][extCounter]==tempExtLocTable[iReceiverID][tempCounter]){
					break;
				} else {
					extCounter++;
				}
			}while(tempExtLocTable[iReceiverID][tempCounter]==-1||ExtLocTable2hop[iReceiverID][extCounter]==-1||extCounter<numVehicles+1);
			tempCounter++;
		}while(tempExtLocTable[iReceiverID][tempCounter]==-1||tempCounter<numVehicles+1);

		//for(int j = 0; j < numVehicles; j++){std::cout << iReceiverID<<":"<< ExtLocTable2hop[iReceiverID][j] << " <-- 2 hops info"<< std::endl;}
		// ==================================================

		================== Not used in Random ========================== */

		Ipv4Address sourceIPadd = InetSocketAddress::ConvertFrom (from).GetIpv4();
		///Vector RecieverPosistion = socket->GetNode()->GetObject<MobilityModel>()->GetPosition();

		// === Extract sender ID from the packet.============
		std::string trim = boost::lexical_cast<std::string>(sourceIPadd);
		std::string trimCount = boost::lexical_cast<std::string>(sourceIPadd);
		boost::erase_head(trim,7); int tempID = boost::lexical_cast<int>(trim);
		if (tempID<10){
			boost::erase_head(trimCount,5); boost::erase_tail(trimCount,2);
		} else if (tempID>=10 && tempID<100){
			boost::erase_head(trimCount,5); boost::erase_tail(trimCount,3);
		} else {
			boost::erase_head(trimCount,5); boost::erase_tail(trimCount,4);
		}
		int CountID = boost::lexical_cast<int>(trimCount);
		int SenderID = ((CountID*256)+tempID)-1;
		// ==================================================


		// === calculate distance between sender and receiver ====
		///float distance = sqrtf((std::pow((SenderPosistion[SenderID].x-RecieverPosistion.x),2))+(std::pow((SenderPosistion[SenderID].y-RecieverPosistion.y),2)));
		ReceivePktCounter[iReceiverID][SenderID]++;

		if(ReceivePktCounter[iReceiverID][SenderID]<numOfPackets){
			if (LastInfoUpdate[iReceiverID][SenderID]!=0){
				Age[iReceiverID][SenderID]+= (Simulator::Now() - LastInfoUpdate[iReceiverID][SenderID]);
				LastInfoUpdate[iReceiverID][SenderID]= Simulator::Now();
			} else {
				LastInfoUpdate[iReceiverID][SenderID]= Simulator::Now();
			}
		}

		//std::cout << Simulator::Now() << ":Receive event, from :"
		//				<< SenderID << ", to: "
		//				<< iReceiverID //<< ", Distance sender & receiver:"
		///				<< distance
		///				<< ", Information age: "<< (informationAge[iReceiverID][SenderID].GetDouble()/1000000)/(ReceivePktCounter[iReceiverID][SenderID]-1) <<"ms"
		//				<< ", Total Messages: " << ReceivePktCounter[iReceiverID][SenderID] << std::endl;

    }
}
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,uint32_t MaxPkt, Time pktInterval, uint32_t pktCount)
{
  std::stringstream msgx;  // packet sequence information;
  int iCarID = socket->GetNode()->GetId();

  if (pktCount <= MaxPkt)
    {
	  Time packteInterval = MilliSeconds(interval);

	  //======= executed once per packet =========
	  if(tempPktCount[iCarID]!=pktCount){
		  sendPayload[iCarID] = PayloadToString(iCarID,numVehicles);
		  packeActivationtTime[iCarID] = Simulator::Now();
		  tempPktCount[iCarID] = pktCount;	// guard
		  if (dGlobalCBR[iCarID][0]<=dGlobalCBR[iCarID][1]&&dGlobalCBR[iCarID][0]<=dGlobalCBR[iCarID][2]){channelAssign[iCarID][pktCount]=1;}
		  else if (dGlobalCBR[iCarID][1]<=dGlobalCBR[iCarID][2]){channelAssign[iCarID][pktCount]=2;}
		  else {channelAssign[iCarID][pktCount]=3;}
		  packetJitter[pktCount] = MicroSeconds(jitter(iCarID,pktCount));
		  Simulator::Schedule (packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
	  } else {
		  // prepare to send.

		  msgx << sendPayload[iCarID]; // Send payload Location Table extention to receiver.

		  if(RandomDistSequence[iCarID][pktCount]==1){

			  if((packeActivationtTime[iCarID]+packteInterval)-Simulator::Now()<MsgDuration){
				DropPktCounter[iCarID][0]++;
				//std::cout << Simulator::Now() << ":Drop event, on VehicleID" << socket->GetNode()->GetId() << ", Msg number:" << pktCount
				//	  <<", total msg drop:" << DropPktCounter[iCarID][0] <<std::endl;
				Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
			  } else if(aifs[iCarID][pktCount][0]>0){
				  aifs[iCarID][pktCount][0]--;
				  if (isPhyBusy[iCarID][0]==true){
					  do_bf[iCarID][pktCount][0]=true;
					  aifs[iCarID][pktCount][0]=Aifs;
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  }
			  } else {
				  if(do_bf[iCarID][pktCount][0]==true && bf[iCarID][pktCount][0]==-1){GenerateBF(iCarID,pktCount,0);}
				  if(isPhyBusy[iCarID][0]==true) {
					  aifs[iCarID][pktCount][0]=Aifs;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else if(bf[iCarID][pktCount][0]>0){
					  bf[iCarID][pktCount][0]--;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  socket->SetAllowBroadcast (true);
					  socket->Connect (InetSocketAddress (Ipv4Address ("10.1.255.255"), port));
					  SenderPosistion[iCarID] = socket->GetNode()->GetObject<MobilityModel>()->GetPosition(); // Record position when sending the packet
					  socket->Send (Create<Packet> ((uint8_t*) msgx.str().c_str(),pktSize));
					  //std::cout << Simulator::Now() << ":Sending event, By VehicleID:" << socket->GetNode()->GetId() << ", Msg number:" << pktCount << std::endl;
					  Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
				  }
			  }

		  } else if(RandomDistSequence[iCarID][pktCount]==2){

			  if((packeActivationtTime[iCarID]+packteInterval)-Simulator::Now()<MsgDuration){
				DropPktCounter[iCarID][1]++;
				//std::cout << Simulator::Now() << ":Drop event, on VehicleID" << socket->GetNode()->GetId() << ", Msg number:" << pktCount
				//	  <<", total msg drop:" << DropPktCounter[iCarID][1] <<std::endl;
				Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
			  } else if(aifs[iCarID][pktCount][1]>0){
				  aifs[iCarID][pktCount][1]--;
				  if (isPhyBusy[iCarID][1]==true){
					  do_bf[iCarID][pktCount][1]=true;
					  aifs[iCarID][pktCount][1]=Aifs;
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  }
			  } else {
				  if(do_bf[iCarID][pktCount][1]==true && bf[iCarID][pktCount][1]==-1){GenerateBF(iCarID,pktCount,1);}
				  if(isPhyBusy[iCarID][1]==true) {
					  aifs[iCarID][pktCount][1]=Aifs;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else if(bf[iCarID][pktCount][1]>0){
					  bf[iCarID][pktCount][1]--;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  socket->SetAllowBroadcast (true);
					  socket->Connect (InetSocketAddress (Ipv4Address ("20.1.255.255"), port));
					  SenderPosistion[iCarID] = socket->GetNode()->GetObject<MobilityModel>()->GetPosition(); // Record position when sending the packet
					  socket->Send (Create<Packet> ((uint8_t*) msgx.str().c_str(),pktSize));
					  //std::cout << Simulator::Now() << ":Sending event, By VehicleID:" << socket->GetNode()->GetId() << ", Msg number:" << pktCount << std::endl;
					  Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
				  }
			  }

		  } else {

			  if((packeActivationtTime[iCarID]+packteInterval)-Simulator::Now()<MsgDuration){
				DropPktCounter[iCarID][2]++;
				//std::cout << Simulator::Now() << ":Drop event, on VehicleID" << socket->GetNode()->GetId() << ", Msg number:" << pktCount
				//	  <<", total msg drop:" << DropPktCounter[iCarID][2] <<std::endl;
				Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
			  } else if(aifs[iCarID][pktCount][2]>0){
				  aifs[iCarID][pktCount][2]--;
				  if (isPhyBusy[iCarID][2]==true){
					  do_bf[iCarID][pktCount][2]=true;
					  aifs[iCarID][pktCount][2]=Aifs;
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  Simulator::Schedule (aifsTimeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  }
			  } else {
				  if(do_bf[iCarID][pktCount][2]==true && bf[iCarID][pktCount][2]==-1){GenerateBF(iCarID,pktCount,2);}
				  if(isPhyBusy[iCarID][2]==true) {
					  aifs[iCarID][pktCount][2]=Aifs;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else if(bf[iCarID][pktCount][2]>0){
					  bf[iCarID][pktCount][2]--;
					  Simulator::Schedule (timeSLot, &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount);
				  } else {
					  socket->SetAllowBroadcast (true);
					  socket->Connect (InetSocketAddress (Ipv4Address ("30.1.255.255"), port));
					  SenderPosistion[iCarID] = socket->GetNode()->GetObject<MobilityModel>()->GetPosition(); // Record position when sending the packet
					  socket->Send (Create<Packet> ((uint8_t*) msgx.str().c_str(),pktSize));
					  //std::cout << Simulator::Now() << ":Sending event, By VehicleID:" << socket->GetNode()->GetId() << ", Msg number:" << pktCount << std::endl;
					  Simulator::Schedule (pktInterval-packetJitter[pktCount], &GenerateTraffic, socket, pktSize, MaxPkt, pktInterval, pktCount+1);
				  }
			  }
		  }

	  }
    }
  else
    {
	  if(iCarID%3==0){
		  for(int i = 0; i<numVehicles;i++){Age[i][iCarID]+= (Simulator::Now()- MilliSeconds(98.5)) - LastInfoUpdate[i][iCarID];}
	  } else if (iCarID%3==1){
		  for(int i = 0; i<numVehicles;i+=3){Age[i][iCarID]+= (Simulator::Now()- MilliSeconds(98.5)) - LastInfoUpdate[i][iCarID];}
		  for(int i = 1; i<numVehicles;i+=3){Age[i][iCarID]+= (Simulator::Now()- MilliSeconds(98.5)) - LastInfoUpdate[i][iCarID];}
	  } else if (iCarID%3==2){
		  for(int i = 0; i<numVehicles;i+=3){Age[i][iCarID]+= (Simulator::Now()- MilliSeconds(98.5)) - LastInfoUpdate[i][iCarID];}
		  for(int i = 2; i<numVehicles;i+=3){Age[i][iCarID]+= (Simulator::Now()- MilliSeconds(98.5)) - LastInfoUpdate[i][iCarID];}
	  }

      socket->Close ();
    }
}

// Trace Layer 1 status
void
PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
	std::string sCarID = context; std::string sChannelID = context;
	boost::erase_head(sCarID,10); boost::erase_tail(sCarID,28); int iCarID = boost::lexical_cast<int>(sCarID);  // extract Node ID
	boost::erase_head(sChannelID,23); boost::erase_tail(sChannelID,15);
	boost::replace_first(sChannelID, "t/", " "); boost::replace_first(sChannelID, "/", " ");boost::erase_all(sChannelID, " ");
	int iChannelID = boost::lexical_cast<int>(sChannelID);  // extract Channel ID

	//std::cout << "receiverID " << iCarID<< "; receive on channel " << iChannelID << std::endl;
	TotalMessageTxRx[iCarID][iChannelID]=TotalMessageTxRx[iCarID][iChannelID]+1;
}
void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
	std::string sCarID = context; std::string sChannelID = context;
	boost::erase_head(sCarID,10); boost::erase_tail(sCarID,26); int iCarID = boost::lexical_cast<int>(sCarID);  // extract Node ID
	boost::erase_head(sChannelID,23); boost::erase_tail(sChannelID,13);
	boost::replace_first(sChannelID, "t/", " "); boost::replace_first(sChannelID, "/", " "); boost::erase_all(sChannelID, " ");
	int iChannelID = boost::lexical_cast<int>(sChannelID);  // extract Channel ID

	//std::cout << "transmitID " << iCarID<< "; receive on channel " << iChannelID << std::endl;
	TotalMessageTxRx[iCarID][iChannelID]=TotalMessageTxRx[iCarID][iChannelID]+1;
	TotalMessageTx[iCarID][iChannelID]=TotalMessageTx[iCarID][iChannelID]+1;
}
void
PhyStateTrace (std::string context, Time start, Time duration, enum WifiPhy::State state)
{
	if ((state == 2 || state == 3) && MsgDuration == 0){
		MsgDuration = duration;
		dMaxMessagePerDuration = CBRprobingInterval/MsgDuration;
		//std::cout << " MaxMessages(per interval) = " << dMaxMessagePerDuration << ", Message transmit time :" << duration << std::endl;
	}
}
void
PhyRXBeginTrace (std::string context, Ptr<const Packet> p)
{
	std::string sCarID = context; std::string sChannelID = context;
	boost::erase_head(sCarID,10); boost::erase_tail(sCarID,48); int iCarID = boost::lexical_cast<int>(sCarID);  // extract Node ID
	boost::erase_head(sChannelID,23); boost::erase_tail(sChannelID,35);
	boost::replace_first(sChannelID, "t/", " "); boost::replace_first(sChannelID, "/", " ");  boost::erase_all(sChannelID, " ");
	int iChannelID = boost::lexical_cast<int>(sChannelID);  // extract Channel ID

	//std::cout << "carID " << iCarID<< "; queue on channel " << iChannelID << std::endl;
	isPhyBusy[iCarID][iChannelID] = true;
}
void
PhyRXEndTrace (std::string context, Ptr<const Packet> p)
{
	std::string sCarID = context; std::string sChannelID = context;
	boost::erase_head(sCarID,10); boost::erase_tail(sCarID,46); int iCarID = boost::lexical_cast<int>(sCarID);  // extract Node ID
	boost::erase_head(sChannelID,23); boost::erase_tail(sChannelID,33);
	boost::replace_first(sChannelID, "t/", " "); boost::replace_first(sChannelID, "/", " ");  boost::erase_all(sChannelID, " ");
	int iChannelID = boost::lexical_cast<int>(sChannelID);  // extract Channel ID

	//std::cout << "carID " << iCarID<< "; queue on channel " << iChannelID << std::endl;
	isPhyBusy[iCarID][iChannelID] = false;
}

// calculating strategy (CBR, Random)
void CBRprobing(Time listenTime, uint32_t vehicles)
{
	uint32_t VecID = vehicles;
	uint32_t CarID = vehicles;

	//==std::clock_t    start;

	if (Simulator::Now() <= Chronos){ // Guarded by Chronos.
		//==start = std::clock();

		do{
			dLocalCBR[VecID-1][0]=TotalMessageTxRx[VecID-1][0]/dMaxMessagePerDuration;
			dLocalCBR[VecID-1][1]=TotalMessageTxRx[VecID-1][1]/dMaxMessagePerDuration;
			dLocalCBR[VecID-1][2]=TotalMessageTxRx[VecID-1][2]/dMaxMessagePerDuration;
			//std::cout << dLocalCBR[VecID-1][0] << ", " << dLocalCBR[VecID-1][1] << ", " << dLocalCBR[VecID-1][2] << ", " << std::endl;
			VecID--;
		}while(VecID>0);

		//=== Check avg local CBR======
		double Cch=0,Sch1=0,Sch2=0;
		for(uint32_t y=0;y<vehicles;y++){
			Cch+=dLocalCBR[y][0];
			Sch1+=dLocalCBR[y][1];
			Sch2+=dLocalCBR[y][2];
		}
		if (Cch/numVehicles!=0||Sch1/numVehicles!=0||Sch2/numVehicles!=0){
		std::cout << Cch/numVehicles <<","<< Sch1/numVehicles <<"," << Sch2/numVehicles << std::endl;
		}

		//std::cout << Simulator::Now() << " calculate Global CBR \n";
		// ======================== CBR Control Channel ==================
		for(uint32_t y=0;y<vehicles;y++){
			dGlobalCBR[y][0]=dLocalCBR[y][0]; // Local CBR as Global CBR
			for(uint32_t x=0;x<vehicles;x++){
				//std::cout<<dLocalCBR[ExtLocTable[y][x+1]][1]<<std::endl;
				if (dGlobalCBR[y][0]<dLocalCBR[ExtLocTable1hop[y][x]][0]){ // Local CBR compare the local to all the 1 hop CBR
					dGlobalCBR[y][0]=dLocalCBR[ExtLocTable1hop[y][x]][0];
				}
			}
			for(uint32_t x=0;x<vehicles;x++){
				//std::cout<<dLocalCBR[ExtLocTable[y][x+1]][1]<<std::endl;
				if (dGlobalCBR[y][0]<dLocalCBR[ExtLocTable2hop[y][x]][0]){ // Local CBR compare the local to all the 2 hop CBR
					dGlobalCBR[y][0]=dLocalCBR[ExtLocTable2hop[y][x]][0];
				}
			}
			//std::cout<<"==========="<<std::endl;
			//std::cout<<Simulator::Now()<<","<<y<<" <--vehicleID, CBR value: "<<dGlobalCBR[y][0]<<std::endl;
			//std::cout<<"==========="<<std::endl;
		}

		// ======================== CBR Service Channel 1 ==================
		for(uint32_t y=0;y<vehicles;y++){
			dGlobalCBR[y][1]=dLocalCBR[y][1]; // Local CBR as Global CBR
			for(uint32_t x=0;x<vehicles;x++){
				if (dGlobalCBR[y][1]<dLocalCBR[ExtLocTable1hop[y][x]][1]){ // Local CBR compare the local to all the 1 hop CBR
					dGlobalCBR[y][1]=dLocalCBR[ExtLocTable1hop[y][x]][1];
				}
			}
			for(uint32_t x=0;x<vehicles;x++){
				//std::cout<<dLocalCBR[ExtLocTable[y][x+1]][1]<<std::endl;
				if (dGlobalCBR[y][1]<dLocalCBR[ExtLocTable2hop[y][x]][1]){ // Local CBR compare the local to all the 2 hop CBR
					dGlobalCBR[y][1]=dLocalCBR[ExtLocTable2hop[y][x]][1];
				}
			}
		}

			// ======================== CBR Service Channel 2 ==================
		for(uint32_t y=0;y<vehicles;y++){
			dGlobalCBR[y][2]=dLocalCBR[y][2]; // Local CBR as Global CBR
			for(uint32_t x=0;x<vehicles;x++){
				if (dGlobalCBR[y][2]<dLocalCBR[ExtLocTable1hop[y][x]][2]){ // Local CBR compare the local to all the 1 hop CBR
					dGlobalCBR[y][2]=dLocalCBR[ExtLocTable1hop[y][x]][2];
				}
			}
			for(uint32_t x=0;x<vehicles;x++){
				//std::cout<<dLocalCBR[ExtLocTable[y][x+1]][2]<<std::endl;
				if (dGlobalCBR[y][2]<dLocalCBR[ExtLocTable2hop[y][x]][2]){ // Local CBR compare the local to all the 2 hop CBR
					dGlobalCBR[y][2]=dLocalCBR[ExtLocTable2hop[y][x]][2];
				}
			}
		}
		/*
		for(uint32_t Vec=0;Vec<vehicles;Vec++){
		std::cout << Simulator::Now() << " Global CBR vehicleID:"<<Vec<<"; (CCH,SCH1,SCH2) = " << dGlobalCBR[Vec][0] <<","<< dGlobalCBR[Vec][1] <<","
				<< dGlobalCBR[Vec][2] << std::endl;
		}*/

		//std::cout << Simulator::Now() << " clear local CBR counter \n";
		do{
			//std::cout << CarID-1 <<"<-carID  "<< TotalMessageTxRx[CarID-1][0] << ", " << TotalMessageTxRx[CarID-1][1] << ", " << TotalMessageTxRx[CarID-1][2] << std::endl;
			TotalMessageTxRx[CarID-1][0]=0;
			TotalMessageTxRx[CarID-1][1]=0;
			TotalMessageTxRx[CarID-1][2]=0;
			CarID--;
		}while(CarID>0);

		// ======  reset tables =======
		for(uint32_t x = 0; x<vehicles; x++){
		 	for(uint32_t y=0; y<vehicles; y++){
		 		ExtLocTable1hop[x][y]=-1;
		 		ExtLocTable2hop[x][y]=-1;
		 		tempExtLocTable[x][y]=-1;
		 	}
		 }
		//== std::cout << Simulator::Now() << "; Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000000) << " micro sec" << std::endl;

		Simulator::Schedule(listenTime, &CBRprobing, listenTime, vehicles);
	}
}

int main (int argc, char *argv[])
{
  uint32_t packetSize = 1020; 			// in bytes.  if you change the packet size, don't forget to change the interval to transmit between vehicles.
  	  	  	  	  	  	  	  			// 1020 simulates 1000 payload for GN SHB. Mac Header (40) + LLC (4) + GN SHB ITS-G5 (40) + Payload (1000) = 1084 byte.
  uint32_t numPackets = 50; 			// Max 1000 msgs, used for Random strategy.
  uint32_t carsNum = 3;  				// Min 2 (1 RSU and 1 vehicle), max 1000 (incld RSU). max for 1-hop urban network is 105 with increasing distance. for initiate.
  double pktinterval = 100.0; 			// in Milliseconds, between 100 - 1000ms according to CAM standard Document
  double CBRmonirotrInterval = 100.0;	// interval time (millisecond) monitor CBR calculation TS 102 636-4-2 and Tiels et al.
  Time guard = Seconds (((pktinterval*numPackets)/1000)+5.0);		     // used as Max time for the CBR calculation and statistic collection.

  CommandLine cmd;

  cmd.AddValue ("numPackets", "number of packets generated(max 1000 for random strategy", numPackets);
  cmd.AddValue ("interval", "interval (Milliseconds) between packets", pktinterval);
  cmd.AddValue ("vehicles", "number of vehicles (max 1000)", carsNum);
  cmd.AddValue ("monitor", "interval time(ms) to monitor the number of Tx/Rx packet", CBRmonirotrInterval);
  cmd.Parse (argc, argv);

  numVehicles = carsNum;
  numOfPackets = numPackets;
  Chronos = guard;
  CBRprobingInterval = MilliSeconds (CBRmonirotrInterval);
  interval = pktinterval;

  // Convert to time object
  Time interPacketInterval = MilliSeconds (interval);

  NodeContainer cars;
  cars.Create (carsNum);		// Number of cars

  // ===============  Setup the Physical layer ===================
  YansWifiChannelHelper Channel;
  Channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

  // Rural Area. Joseph Benin et al.
  Channel.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel",
		  "Distance1", DoubleValue(210.0), "Exponent1", DoubleValue(15.0),
		  "Distance2", DoubleValue(286.0), "Exponent2", DoubleValue(3.65));

  Channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel",
		  "Distance2", DoubleValue(320.0),
		  "m2", DoubleValue(0));

  YansWifiPhyHelper cchPhy =  YansWifiPhyHelper::Default ();
  cchPhy.Set("TxGain", DoubleValue(0));
  cchPhy.Set("RxGain", DoubleValue(0));
  cchPhy.Set("EnergyDetectionThreshold", DoubleValue (-99.0) );
  cchPhy.Set("CcaMode1Threshold", DoubleValue (-95.0) );
  cchPhy.Set("TxPowerStart", DoubleValue(23.0));
  cchPhy.Set("TxPowerEnd", DoubleValue(23.0));
  cchPhy.Set("RxNoiseFigure", DoubleValue(8.0));
  cchPhy.Set("ChannelNumber", UintegerValue (180));
  Ptr<YansWifiChannel> cchChannel = Channel.Create ();
  cchPhy.SetChannel (cchChannel);

  YansWifiPhyHelper sch1Phy =  YansWifiPhyHelper::Default ();
  sch1Phy.Set("TxGain", DoubleValue(0));
  sch1Phy.Set("RxGain", DoubleValue(0));
  sch1Phy.Set("EnergyDetectionThreshold", DoubleValue (-99.0) );
  sch1Phy.Set("CcaMode1Threshold", DoubleValue (-95.0) );
  sch1Phy.Set("TxPowerStart", DoubleValue(23.0));
  sch1Phy.Set("TxPowerEnd", DoubleValue(23.0));
  sch1Phy.Set("RxNoiseFigure", DoubleValue(8.0));
  sch1Phy.Set("ChannelNumber", UintegerValue (176));
  Ptr<YansWifiChannel> sch1Channel = Channel.Create ();
  sch1Phy.SetChannel (sch1Channel);

  YansWifiPhyHelper sch2Phy =  YansWifiPhyHelper::Default ();
  sch2Phy.Set("TxGain", DoubleValue(0));
  sch2Phy.Set("RxGain", DoubleValue(0));
  sch2Phy.Set("EnergyDetectionThreshold", DoubleValue (-99.0) );
  sch2Phy.Set("CcaMode1Threshold", DoubleValue (-95.0) );
  sch2Phy.Set("TxPowerStart", DoubleValue(23.0));
  sch2Phy.Set("TxPowerEnd", DoubleValue(23.0));
  sch2Phy.Set("RxNoiseFigure", DoubleValue(8.0));
  sch2Phy.Set("ChannelNumber", UintegerValue (178));
  Ptr<YansWifiChannel> sch2Channel = Channel.Create ();
  sch2Phy.SetChannel (sch2Channel);

  // ============  Setup the MAC layer =================
  // https://www.nsnam.org/docs/models/html/wave.html
  // The Wifi80211pHelper is used to create 802.11p devices that follow the 802.11p-2010 standard.
  // The WaveHelper is used to create WAVE devices that follow both 802.11p-2010 and 1609.4-2010 standards which are the MAC and PHY layers of the WAVE architecture.
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  // The Wifi80211pHelper ------use----->  QosWaveMacHelper or NqosWaveHelper that inherit QosWifiMacHelper or NqosWifiMacHelper (respectively).
  // The WaveHelper -------- only use --------> QosWaveMacHelper that inherit QosWifiMacHelper.
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default();
  wifi80211p.SetStandard(WIFI_PHY_STANDARD_80211_10MHZ);

  NetDeviceContainer cchDevices = wifi80211p.Install (cchPhy, wifi80211pMac, cars);
  NetDeviceContainer sch1Devices = wifi80211p.Install (sch1Phy, wifi80211pMac, cars);
  NetDeviceContainer sch2Devices = wifi80211p.Install (sch2Phy, wifi80211pMac, cars);

  // ===========  Setup the Network layer ================
  InternetStackHelper internet;
  internet.Install (cars);

  Ipv4AddressHelper ipv4cch, ipv4sch1, ipv4sch2;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4cch.SetBase ("10.1.0.0", "255.255.0.0");
  ipv4sch1.SetBase ("20.1.0.0", "255.255.0.0");
  ipv4sch2.SetBase ("30.1.0.0", "255.255.0.0");
  Ipv4InterfaceContainer iCCH = ipv4cch.Assign (cchDevices);
  Ipv4InterfaceContainer iSCH1 = ipv4sch1.Assign (sch1Devices);
  Ipv4InterfaceContainer iSCH2 = ipv4sch2.Assign (sch2Devices);

  // ============  Setup the Grid. =====================
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  uint32_t vehiclesPosition = 0;
  uint32_t distanceVehicle = 0;

  while (vehiclesPosition<carsNum){		// Vehicles positions
	  if(vehiclesPosition % 12 == 0){
		  positionAlloc->Add (Vector (10.0, (65.0-(distanceVehicle*5)), 0.0));
	  }
	  else if (vehiclesPosition % 12 == 1){
		  positionAlloc->Add (Vector (190.0, (75.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 2){
		  positionAlloc->Add (Vector (-190.0, (-75.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 3){
		  positionAlloc->Add (Vector (-10.0, (-65.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 4){
		  positionAlloc->Add (Vector (185.0, (70.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 5){
		  positionAlloc->Add (Vector (-185.0, (-70.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 6){
		  positionAlloc->Add (Vector (5.0, (70.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 7){
		  positionAlloc->Add (Vector (180.0, (65.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 8){
		  positionAlloc->Add (Vector (-180.0, (-65.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 9){
		  positionAlloc->Add (Vector (-5.0, (-70.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 10){
		  positionAlloc->Add (Vector (175.0, (60.0-(distanceVehicle*5)), 0.0));
	  }
	  else if(vehiclesPosition % 12 == 11){
		  positionAlloc->Add (Vector (-175.0, (-60.0-(distanceVehicle*5)), 0.0));
	  	  distanceVehicle++;
	  }
	  vehiclesPosition++;
  }

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (cars);

  uint32_t vehiclesVelocity = 0;
  Ptr<ConstantVelocityMobilityModel> motion[carsNum];
  while (vehiclesVelocity<carsNum){		// Vehicles velocity
 	  if (vehiclesVelocity % 12 == 1){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -20.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 2){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 20.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 3){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 10.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 4){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -15.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 5){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 15.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 6){
 		motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 		motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -15.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 7){
 	  	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	  	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -10.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 8){
 	  	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	  	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 10.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 9){
 	  	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	  	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 15.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 10){
 	  	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	  	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -5.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 11){
 	  	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	  	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, 5.0, 0.0));
 	  }
 	  else if(vehiclesVelocity % 12 == 0){
 	 	motion[vehiclesVelocity] = DynamicCast<ConstantVelocityMobilityModel>( cars.Get(vehiclesVelocity)->GetObject<MobilityModel>());
 	 	motion[vehiclesVelocity]->SetVelocity(Vector(0.0, -10.0, 0.0));
 	  }
 	 vehiclesVelocity++;
   }

  // =============  Setup the CAM Application ==============
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // cam Sender
  Ptr<Socket> source[carsNum];
  uint32_t senderCounter = 0;
  while(senderCounter < carsNum){
	  source[senderCounter] = Socket::CreateSocket (cars.Get (senderCounter), tid);
	  senderCounter++;
  }

  // cam Receiver
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  Ptr<Socket> recvSink[carsNum];
  uint32_t recvCounter = 0;

  while(recvCounter<carsNum){
	  recvSink[recvCounter] = Socket::CreateSocket (cars.Get (recvCounter), tid);
	  recvSink[recvCounter]->Bind (local);
	  recvSink[recvCounter]->SetRecvCallback (MakeCallback (&ReceivePacket));
	  recvCounter++;
  }

  // =====  Tracing during simulation =======
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback (&PhyRxOkTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));					  // remove message from queue
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace));				  // calculate Tx and Rx active time for a packet.
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback(&PhyRXBeginTrace));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", MakeCallback(&PhyRXEndTrace));

  // ============= Simulation ======================

  // reset tables
  	for(uint32_t x = 0; x<carsNum; x++){
  			for(uint32_t y=0; y<carsNum; y++){
  				ExtLocTable1hop[x][y]=-1;
  				ExtLocTable2hop[x][y]=-1;
  				tempExtLocTable[x][y]=-1;
  			}
  		}
  // reset pkt payload guard
  	for(uint32_t z=0; z<carsNum; z++){
  		tempPktCount[z]=-1;
  	}

  // reset statistic counter
  	for (int i=0; i<numVehicles; i++){
  		SMR[i] = 0.0;
  	}

  // reset CSMA/CA parameters for broadcast
    for(uint32_t i=0;i<carsNum;i++){
    	for(uint32_t j=1;j<=numPackets;j++){
    		for(uint32_t k=0;k<3;k++){
    			aifs[i][j][k]=Aifs;
    			bf[i][j][k]=-1;
    			do_bf[i][j][k]=false;
    		}
    	}
    }

  // ============================================

  uint32_t vehicleID=0;
  double executeTime;
  int PktSequence=1;

  // ================= Not used in Random =================
  //Simulator::Schedule(Seconds (1.000001),&CBRprobing, CBRprobingInterval, carsNum); // trigger CBR value.

  do{
	  executeTime=1.0+(phasing(vehicleID)/1000000000);  //<===== Simulation start!!
	  Simulator::ScheduleWithContext (source[vehicleID]->GetNode()->GetId(), Seconds (executeTime),
			  &GenerateTraffic, source[vehicleID], packetSize, numPackets, interPacketInterval, PktSequence);
	  vehicleID++;

  }while(vehicleID<carsNum);

  uint32_t random = carsNum;
  for(;random>=1;random--){randomGenerator(numPackets, random);}

  Simulator::Schedule(Chronos,&statisticData);
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
