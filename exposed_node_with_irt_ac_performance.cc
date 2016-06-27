#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/snr-tag.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/string.h"
#include "ns3/netanim-module.h"
#include "ns3/command-line.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <ns3/spectrum-helper.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/random-variable-stream.h>
#include <ns3/config-store.h>
#include <ns3/constant-velocity-mobility-model.h>
#include "ns3/rng-seed-manager.h"
using namespace ns3;

uint32_t PktTxCount [255];
uint32_t seed = 1;
uint32_t run = 1;
uint32_t no_nodes;
double arrival_times[100];

class CustomHeader : public Header 
{
public:

  CustomHeader ();
  virtual ~CustomHeader ();

  void SetData (uint32_t data);
  uint32_t GetData (void) const;
  void SetSrcID (uint32_t src_id);
  void SetDstID (uint32_t dst_id);
  uint32_t GetSrcID ();
  uint32_t GetDstID ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
private:
  uint32_t m_data;
  uint32_t m_src_id;
  uint32_t m_dst_id;
};

CustomHeader::CustomHeader () {}

CustomHeader::~CustomHeader () {}

TypeId CustomHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CustomHeader")
    .SetParent<Header> ()
    .AddConstructor<CustomHeader> ()
  ;
  return tid;
}
TypeId CustomHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
void CustomHeader::Print (std::ostream &os) const
{
  os << "data= " << m_data <<" src_id= "<<m_src_id<<" dst_id= "<<m_dst_id;
}
uint32_t CustomHeader::GetSerializedSize (void) const
{
  return 12;
}
void CustomHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteHtonU32 (m_data);
  start.WriteHtonU32 (m_src_id);
  start.WriteHtonU32 (m_dst_id);
}
uint32_t CustomHeader::Deserialize (Buffer::Iterator start)
{
  m_data = start.ReadNtohU32 ();
  m_src_id = start.ReadNtohU32 ();
  m_dst_id = start.ReadNtohU32 ();
  return 12;
}
void CustomHeader::SetData (uint32_t data)
{
  m_data = data;
}
uint32_t CustomHeader::GetData (void) const
{
  return m_data;
}
void CustomHeader::SetSrcID (uint32_t src_id)
{
  m_src_id = src_id;
}
void CustomHeader::SetDstID (uint32_t dst_id)
{
  m_dst_id = dst_id;
}
uint32_t CustomHeader::GetSrcID ()
{
  return m_src_id;
}
uint32_t CustomHeader::GetDstID ()
{
  return m_dst_id;
}

//-------------------------------------------------------Trace Functions--------------------------------------------------
static void
PhyCallbackTrace (std::string path, Ptr<const Packet> packet)
{
  //std::cout<<path<<std::endl;
  //std::cout<<Now ().GetSeconds ()<<std::endl; //Print the time at which transmission just begins
  uint32_t delta = path.substr(10).find_first_of("/");
  std::string sub_string = path.substr(10, delta);
  uint32_t node_no;
  std::stringstream ss;
  ss << sub_string;
  ss >> node_no;
 // if (node_no == 0 || node_no == 1 || node_no == no_nodes )
//	std::cout<<"Node "<<node_no<<" attempting to transmit at t = "<<Now ().GetSeconds()<<"s\n";
  //if (node_no == 0)
	//PktTxCount++;
  PktTxCount[node_no] = PktTxCount[node_no] + 1;
}

static void
PhyRxCallbackTrace (std::string path, Ptr<const Packet> packet)
{
  //std::cout<<path<<std::endl;
  //std::cout<<Now ().GetSeconds ()<<std::endl; //Print the time at which transmission just begins
  uint32_t delta = path.substr(10).find_first_of("/");
  std::string sub_string = path.substr(10, delta);
  uint32_t node_no;
  std::stringstream ss;
  ss << sub_string;
  ss >> node_no;
  //std::cout<<"Node "<<node_no<<" started receiving at t = "<<Now ().GetSeconds()<<"s\n";
}

static void
MacTxDrop (std::string path, Ptr<const Packet> packet)
{
   std::cout<<"In PktDrop trace sink\n";
}

//-------------------------------------------------------DSRC Nodes-------------------------------------------------------

class DsrcNodes
 {
public:
  void SendWsmpExample ();
  void Initialize (uint32_t vehicle_density, uint32_t no_lanes, uint32_t road_length, uint32_t lane_separation, Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, double interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double m_RxGain, double m_TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t slot, double Sensitivity, double CcaSensitivity);
  bool mobility;
  uint32_t m_rxPacketCounter;  

private:
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq, uint32_t node_id);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void CreateWaveNodes ();
 
  uint32_t m_noNodes;
  uint32_t m_noPackets;
  uint32_t m_packetSize;
  std::vector <double> m_txSafetyRanges;
  double m_gpsAccuracyNs;
  double m_interval;
  int64_t m_streamIndex;
  uint32_t m_simTime;
  int m_nodeSpeed; //in m/s
  int m_nodePause; //in s
  double m_distanceToRx;
  double m_RxGain;
  double m_TxGain;
  double m_txSafetyRange1;
  double m_txSafetyRange2;
  double m_txSafetyRange3;
  double m_txSafetyRange4;
  double m_txSafetyRange5;
  double m_txSafetyRange6;
  double m_txSafetyRange7;
  double m_txSafetyRange8;
  double m_txSafetyRange9;
  double m_txSafetyRange10;
  double m_freq;
  uint32_t m_sifs;
  uint32_t m_pifs; 
  uint32_t m_eifsnodifs;
  uint32_t m_vehicle_density;
  uint32_t m_road_length;
  uint32_t m_no_lanes;
  uint32_t m_lane_separation;
  uint32_t m_slot; 
  double m_sensitivity;
  double m_cca_sensitivity;
  Ptr<YansWifiChannel> m_channel;
  WaveBsmHelper m_waveBsmHelper;
  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
DsrcNodes::Initialize (uint32_t vehicle_density, uint32_t no_lanes, uint32_t road_length, uint32_t lane_separation, Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, double interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double RxGain, double TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t slot, double Sensitivity, double CcaSensitivity)
{ 
  m_noNodes = vehicle_density*road_length;
  m_vehicle_density = vehicle_density;
  m_road_length = road_length;
  m_no_lanes = no_lanes;
  m_lane_separation = lane_separation;
  m_rxPacketCounter = 0;
  m_distanceToRx = distanceToRx;
  m_noPackets = noPackets;
  m_packetSize = packetSize;
  m_gpsAccuracyNs = gpsAccuracyNs;
  m_interval = interval;
  m_simTime = simTime;
  m_nodeSpeed = nodeSpeed;
  m_nodePause = nodePause;
  m_RxGain = RxGain;
  m_TxGain = TxGain;
  m_freq = freq;
  m_channel = wirelessChannel;
  m_sifs = sifs;
  m_pifs = pifs;
  m_eifsnodifs = eifsnodifs;
  m_slot = slot;
  m_sensitivity = Sensitivity;
  m_cca_sensitivity = CcaSensitivity;

  double txDist1 = 50.0;
  double txDist2 = 100.0;
  double txDist3 = 150.0;
  double txDist4 = 200.0;
  double txDist5 = 250.0;
  double txDist6 = 300.0;
  double txDist7 = 350.0;
  double txDist8 = 350.0;
  double txDist9 = 350.0;
  double txDist10 = 350.0;

  m_txSafetyRange1 = txDist1;
  m_txSafetyRange2 = txDist2;
  m_txSafetyRange3 = txDist3;
  m_txSafetyRange4 = txDist4;
  m_txSafetyRange5 = txDist5;
  m_txSafetyRange6 = txDist6;
  m_txSafetyRange7 = txDist7;
  m_txSafetyRange8 = txDist8;
  m_txSafetyRange9 = txDist9;
  m_txSafetyRange10 = txDist10;

  m_txSafetyRanges.resize (10, 0);
  m_txSafetyRanges[0] = m_txSafetyRange1;
  m_txSafetyRanges[1] = m_txSafetyRange2;
  m_txSafetyRanges[2] = m_txSafetyRange3;
  m_txSafetyRanges[3] = m_txSafetyRange4;
  m_txSafetyRanges[4] = m_txSafetyRange5;
  m_txSafetyRanges[5] = m_txSafetyRange6;
  m_txSafetyRanges[6] = m_txSafetyRange7;
  m_txSafetyRanges[7] = m_txSafetyRange8;
  m_txSafetyRanges[8] = m_txSafetyRange9;
  m_txSafetyRanges[9] = m_txSafetyRange10;

}

void
DsrcNodes::CreateWaveNodes ()
{
  if (!mobility)
  {
  	m_noNodes=2;	
  }
  nodes = NodeContainer ();
  nodes.Create (m_noNodes);
 
  uint32_t coverage_area_node_count = 0;
 
  if (!mobility)
  {
  //std::cout<<"WAVE Devices following constant mobility model\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (500-m_distanceToRx, 400.0, 0.0));
  positionAlloc->Add (Vector (500, 400.0, 0.0));
  //positionAlloc->Add (Vector (500+2*m_distanceToRx, 25.0, 0.0)); //Comment this out for two nodes
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  }
  else
  {
  //mobility starts here............................................................................
  //std::cout<<"WAVE Devices following highway mobility model\n";
  MobilityHelper mobility;
  Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable> ();
  xPos->SetAttribute ("Min", DoubleValue(0.0));
  xPos->SetAttribute ("Max", DoubleValue(m_road_length*1000));
  //xPos->SetAttribute ("Min", DoubleValue(450.0));
  //xPos->SetAttribute ("Max", DoubleValue(550.0));
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (270, 150.0, 0.0));
  positionAlloc->Add (Vector (100, 150.0, 0.0));
  
  double rand_x_cord;
  for (uint32_t i = 0; i < m_no_lanes; ++i){
  	for (uint32_t j = 0; j < (m_noNodes/m_no_lanes); ++j){
                rand_x_cord = xPos->GetValue ();
		if (rand_x_cord >= 200 && rand_x_cord <=800){
			coverage_area_node_count++;
		}
  		positionAlloc->Add (Vector (rand_x_cord, 144.0+(i*m_lane_separation), 0.0));
	}
  }
  mobility.SetPositionAllocator (positionAlloc);

  //std::cout<<"Assuming coverage area = 300, number of nodes in coverage area of node 1 = "<<coverage_area_node_count<<std::endl<<std::endl;
  //---------------------------------------------------------------------------------------------------
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (nodes);
  Ptr<UniformRandomVariable> rvar = CreateObject<UniformRandomVariable>();
  uint32_t node_count = 0;
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i){
          node_count++;
	  Ptr<Node> node = (*i);
	  //double speed = rvar->GetValue(5, 10);
          double speed = 0.0;
	  if (node_count <= m_noNodes/2)
	  	node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed,0,0));
	  else 
	  	node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(-speed,0,0));
  }
  //mobility ends here............................................................................
  }
  //Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(1)); //Set DSRC Node Queue Size to 1

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.Set ("TxGain", DoubleValue (m_TxGain) );
  wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
  wavePhy.Set ("EnergyDetectionThreshold", DoubleValue (m_sensitivity));
  wavePhy.Set ("CcaMode1Threshold",DoubleValue(m_cca_sensitivity));
  wavePhy.Set ("Frequency", UintegerValue (m_freq));
  wavePhy.SetChannel (m_channel);
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  waveMac.SetType("ns3::OcbWifiMac","Pifs",TimeValue(MicroSeconds(m_pifs)));
  waveMac.SetType("ns3::OcbWifiMac","Sifs",TimeValue(MicroSeconds(m_sifs)));
  waveMac.SetType("ns3::OcbWifiMac","EifsNoDifs",TimeValue(MicroSeconds(m_eifsnodifs)));
  waveMac.SetType("ns3::OcbWifiMac","Slot",TimeValue(MicroSeconds(m_slot)));
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
  PointerValue ptr;

  Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(400)); //Set Interferer queue size back to default

  for (uint32_t i = 0; i != devices.GetN (); ++i)
     {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&DsrcNodes::Receive, this));
     }

  
  InternetStackHelper internet;
  internet.Install (nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
 
   // Tracing
  //wavePhy.EnablePcag ("wave-simple-device", devices);
}

void
DsrcNodes::SendWsmpExample ()
{
  CreateWaveNodes ();
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  // Alternating access without immediate channel switch

  //for (uint32_t i=0; i<1; i++){ //Enable only one transmitter
  for (uint32_t i=0; i<m_noNodes; i++){ //Enable all transmitters
  //      if (i==1){
  //		continue; //To test sensing range, center node does not transmit 
  //	}
	Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (i));
  	Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (i));
  	Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  	Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,receiver,schInfo); // An important point is that the receiver should also be assigned channel access for the same channel to receive packets.
  }

  RngSeedManager::SetSeed (seed);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (run);   // Changes run number from default of 1 to 7
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.004));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.01));  
  //startTimeSeconds->GetValue ();

  //for (uint32_t node_no=0; node_no<1; node_no++){ ////Enable only one transmitter
  for (uint32_t node_no=0; node_no<m_noNodes; node_no++){ //Enable all transmitters
  //      if (node_no == 1){
  //		continue; //For testing sensing range, center node does not transmit
  //	}
        double temp = startTimeSeconds->GetValue ();
	for (uint32_t i=1; i<= m_noPackets; i++)
  	{ 
   	//std::cout<<"Node Number = "<<node_no<<"\tjitter = "<<temp<<"\tScheduled time = "<<(temp + m_interval*i)<<std::endl;
	Simulator::Schedule (Seconds (temp + m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+i, node_no); //With jitter only at the beginning
   	//Simulator::Schedule (Seconds (startTimeSeconds->GetValue () + m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+i, node_no); //With jitter in each IBS
	//Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+i, node_no); //Without jitter

  	/*
        Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+5*i-4, node_no);
   	Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+5*i-3, node_no);
   	Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+5*i-2, node_no);
   	Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+5*i-1, node_no);
   	Simulator::Schedule (Seconds (m_interval*i), &DsrcNodes::SendOneWsmpPacket,  this, CCH, (node_no)*1000+5*i, node_no);
  //	*/
        }
  }
  Config::Connect ("NodeList/*/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/MacTxDrop", MakeCallback(&MacTxDrop));
  //Config::Connect ("NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/PhyTxBegin",MakeCallback(&PhyCallbackTrace));
}

bool
DsrcNodes::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  CustomHeader destinationHeader;
  Ptr<Packet> p = pkt->Copy ();
  p->RemoveHeader (seqTs);
  p->RemoveHeader (destinationHeader);
  SnrTag tag;
  uint32_t cur_node, src_node;
 
    if (seqTs.GetSeq() < 10000000) //DSRC receiver receives only DSRC packets
    //if (true) //DSRC Receiver receives Interferer as well as DSRC packets 
      {  
      cur_node = dev->GetNode ()->GetId();
      src_node = destinationHeader.GetSrcID ();
      if (cur_node == 1 && src_node == 0){ //Only looking at packets sent from node 0 to node 1
	      arrival_times[m_rxPacketCounter]=Now ().GetSeconds ();
	      m_rxPacketCounter++;
///* 
    	      std::cout << "WAVE node "<<cur_node<<" received a packet from node "<<src_node <<" :"
                        << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s,"
            	 	<< "  recvTime = " << Now ().GetSeconds () << "s"
		        << "  SNR = " << tag.Get() 
            		<< "  sequence = " << seqTs.GetSeq () <<std::endl;
//*/	 
      }
     }
  return true;
}
 
void
DsrcNodes::SendOneWsmpPacket  (uint32_t channel, uint32_t seq, uint32_t node_id)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (node_id));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
  const TxInfo txInfo = TxInfo (channel);
  
  Ptr<UniformRandomVariable> pkt_length_rv = CreateObject<UniformRandomVariable> ();
  pkt_length_rv->SetAttribute ("Min", DoubleValue (0.9));
  pkt_length_rv->SetAttribute ("Max", DoubleValue (1.0));
  uint32_t pkt_length = (int) (pkt_length_rv->GetValue () * m_packetSize);  
  //std::cout<<"Packet Length = "<<pkt_length<<std::endl;
  Ptr<Packet> p  = Create<Packet> (pkt_length);
  
  //Ptr<Packet> p  = Create<Packet> (m_packetSize);
  CustomHeader sourceHeader;
  sourceHeader.SetSrcID (node_id);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (sourceHeader);
  p->AddHeader (seqTs);
  sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
}

//-------------------------------------------------------Interferer Nodes-------------------------------------------------------


class Interferer
{
public:
  void SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double m_RxGain, double m_TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t slot, double Sensitivity, double CcaSensitivity);
  bool mobility;
  uint32_t m_rxPacketCounter;  
  bool iverbose;

private:
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void CreateWaveNodes ();

  uint32_t m_noPackets;
  uint32_t m_packetSize;
  std::vector <double> m_txSafetyRanges;
  double m_gpsAccuracyNs;
  float m_interval;
  int64_t m_streamIndex;
  uint32_t m_simTime;
  int m_nodeSpeed; //in m/s
  int m_nodePause; //in s
  double m_distanceToRx;
  double m_RxGain;
  double m_TxGain;
  double m_txSafetyRange1;
  double m_txSafetyRange2;
  double m_txSafetyRange3;
  double m_txSafetyRange4;
  double m_txSafetyRange5;
  double m_txSafetyRange6;
  double m_txSafetyRange7;
  double m_txSafetyRange8;
  double m_txSafetyRange9;
  double m_txSafetyRange10;
  double m_freq;
  uint32_t m_sifs;
  uint32_t m_pifs;  
  uint32_t m_eifsnodifs;
  uint32_t m_slot; 
  double m_sensitivity;
  double m_cca_sensitivity;
  
  Ptr<YansWifiChannel> m_channel;
  WaveBsmHelper m_waveBsmHelper;
  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
Interferer::CreateWaveNodes ()
{
  nodes = NodeContainer ();
  nodes.Create (2);
 
  if (!mobility)
  {
  //std::cout<<"WAVE Devices following constant mobility model\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (500, 0.0, 0.0));
  positionAlloc->Add (Vector (500, -200.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  }
  else
  {
  //mobility starts here............................................................................
  //mobility ends here............................................................................
  }

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.Set ("TxGain", DoubleValue (m_TxGain) );
  wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
  wavePhy.Set ("Frequency", UintegerValue (m_freq));
  wavePhy.Set ("EnergyDetectionThreshold", DoubleValue (m_sensitivity));
  wavePhy.Set ("CcaMode1Threshold",DoubleValue(m_cca_sensitivity));
  wavePhy.SetChannel (m_channel);
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  waveMac.SetType("ns3::OcbWifiMac","Pifs",TimeValue(MicroSeconds(m_pifs)));
  waveMac.SetType("ns3::OcbWifiMac","Sifs",TimeValue(MicroSeconds(m_sifs)));
  waveMac.SetType("ns3::OcbWifiMac","Slot",TimeValue(MicroSeconds(m_slot)));
  waveMac.SetType("ns3::OcbWifiMac","EifsNoDifs",TimeValue(MicroSeconds(m_eifsnodifs)));
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
 
  //---------------------------------------------------Set Contention Window values-------------------------------------------------- 
  std::ostringstream oss;
  for (uint32_t i=0; i<no_nodes; i++){
        oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/DcaTxop/$ns3::DcaTxop/MinCw";
        Config::Set(oss.str(),UintegerValue(127)); //Set MinCW of DSRC to 127
        oss.str("");
	oss.clear();
	oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/VI_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
	Config::Set(oss.str(),UintegerValue(127)); //Set MinCW of DSRC to 127
        oss.str("");
        oss.clear();
	oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/VO_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(127)); //Set MinCW of DSRC to 127
        oss.str("");
        oss.clear();
	oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/BE_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(127)); //Set MinCW of DSRC to 127, actually impacts
        oss.str("");
        oss.clear();
	oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/BK_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(127)); //Set MinCW of DSRC to 127
        oss.str("");
        oss.clear();
  }

  for (uint32_t i=no_nodes; i<=(no_nodes+1); i++){
	oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/DcaTxop/$ns3::DcaTxop/MinCw";
        Config::Set(oss.str(),UintegerValue(0)); //Set MinCW of Interferer to 0
        oss.str("");
        oss.clear();
        oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/VI_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(0)); //Set MinCW of Interferer to 0
        oss.str("");
        oss.clear();
        oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/VO_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(0)); //Set MinCW of Interferer to 0
        oss.str("");
        oss.clear();
        oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/BE_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(0)); //Set MinCW of Interferer to 0, actually impacts
        oss.str("");
        oss.clear();
        oss<<"/$ns3::NodeListPriv/NodeList/"<<i<<"/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/BK_EdcaTxopN/$ns3::EdcaTxopN/MinCw";
        Config::Set(oss.str(),UintegerValue(0)); //Set MinCW of Interferer to 0
        oss.str("");
        oss.clear();
  }
  
  //Config::Set("/$ns3::NodeListPriv/NodeList/2/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/DcaTxop/$ns3::DcaTxop/MinCw",UintegerValue(75));  
  //Config::Set("/$ns3::NodeListPriv/NodeList/3/$ns3::Node/DeviceList/*/$ns3::WaveNetDevice/MacEntities/*/$ns3::OcbWifiMac/DcaTxop/$ns3::DcaTxop/MinCw",UintegerValue(71));  
  
  //--------------------------------------------------------------------------------------------------------------------------------

  PointerValue ptr;
  for (uint32_t i = 0; i != devices.GetN (); ++i)
     {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      Ptr<OcbWifiMac> mac = device->GetMac (172);
      mac->GetAttribute ("DcaTxop",ptr);
      Ptr<DcaTxop> dca = ptr.Get<DcaTxop> ();
      Ptr<WifiMacQueue> queue = dca->GetQueue ();
      //`queue->SetMaxSize (2);
      device->SetReceiveCallback (MakeCallback (&Interferer::Receive, this));
      
     }
  
  InternetStackHelper internet;
  internet.Install (nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.2.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
 
  //Tracing
  //wavePhy.EnablePcap ("wave-simple-device", devices);
}

bool
Interferer::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  SnrTag tag;
  uint32_t cur_node;

  if (seqTs.GetSeq() > 10000000){
  	m_rxPacketCounter++;
    	if(iverbose)	{  // Interference receiver receives only Interferer packets
      	  //if(true)	{  //Interference receiver receives Interferer as well as DSRC packets
		cur_node = dev->GetNode ()->GetId(); 
		std::cout << "Interference receiver "<<cur_node<<" received a packet:"<< "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s,"
                     << "  recvTime = " << Now ().GetSeconds () << "s"
                     << "  SNR = " << tag.Get() 
                     << "  sequence = " << seqTs.GetSeq () << std::endl;
        } 
  }
  return true;
}
 
void
Interferer::SendOneWsmpPacket  (uint32_t channel, uint32_t seq)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
 
  const TxInfo txInfo = TxInfo (channel);
  Ptr<Packet> p  = Create<Packet> (m_packetSize);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
}

void
Interferer::SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double RxGain, double TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t slot, double Sensitivity, double CcaSensitivity)
{
  m_rxPacketCounter = 0;
  m_distanceToRx = distanceToRx;
  m_noPackets = noPackets;
  m_packetSize = packetSize;
  m_gpsAccuracyNs = gpsAccuracyNs;
  m_interval = interval;
  m_simTime = simTime;
  m_nodeSpeed = nodeSpeed;
  m_nodePause = nodePause;
  m_RxGain = RxGain;
  m_TxGain = TxGain;
  m_freq = freq;
  m_channel = wirelessChannel;
  m_sifs = sifs;
  m_pifs = pifs; 
  m_eifsnodifs = eifsnodifs;
  m_slot = slot; 
  m_sensitivity = Sensitivity;
  m_cca_sensitivity = CcaSensitivity;
 
  double txDist1 = 50.0;
  double txDist2 = 100.0;
  double txDist3 = 150.0;
  double txDist4 = 200.0;
  double txDist5 = 250.0;
  double txDist6 = 300.0;
  double txDist7 = 350.0;
  double txDist8 = 350.0;
  double txDist9 = 350.0;
  double txDist10 = 350.0;

  m_txSafetyRange1 = txDist1;
  m_txSafetyRange2 = txDist2;
  m_txSafetyRange3 = txDist3;
  m_txSafetyRange4 = txDist4;
  m_txSafetyRange5 = txDist5;
  m_txSafetyRange6 = txDist6;
  m_txSafetyRange7 = txDist7;
  m_txSafetyRange8 = txDist8;
  m_txSafetyRange9 = txDist9;
  m_txSafetyRange10 = txDist10;

  m_txSafetyRanges.resize (10, 0);
  m_txSafetyRanges[0] = m_txSafetyRange1;
  m_txSafetyRanges[1] = m_txSafetyRange2;
  m_txSafetyRanges[2] = m_txSafetyRange3;
  m_txSafetyRanges[3] = m_txSafetyRange4;
  m_txSafetyRanges[4] = m_txSafetyRange5;
  m_txSafetyRanges[5] = m_txSafetyRange6;
  m_txSafetyRanges[6] = m_txSafetyRange7;
  m_txSafetyRanges[7] = m_txSafetyRange8;
  m_txSafetyRanges[8] = m_txSafetyRange9;
  m_txSafetyRanges[9] = m_txSafetyRange10;

  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
 
  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo); // An important point is that the receiver should also be assigned channel access for the same channel to receive packets.
  
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.05));
  //startTimeSeconds->GetValue ();
  
  for (uint32_t i=1; i<= m_noPackets; i++)
  { 
    //Simulator::Schedule (Seconds (startTimeSeconds->GetValue () + m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i-1);
    //Simulator::Schedule (Seconds (startTimeSeconds->GetValue () + m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i);
    for (uint32_t j=1; j<=50; j++){ //Currently Packet expiration is not implemented. All packets generated at t=0
				    //will be stored in the queue till they are sent. At the current data rate, only 24 
				    //packets can be transmitted before t=0.5.  So transmitting only 24 packets.
	Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 10000000+j*1000+i);
    	//Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 10000000+j*1000+2*i-1);
    }
    //Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000+2*i-1);
    //Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, SCH1, 1000+2*i);
  }
   
  Config::Connect ("NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/PhyTxBegin",MakeCallback(&PhyCallbackTrace));
  Config::Connect ("NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/PhyRxBegin",MakeCallback(&PhyRxCallbackTrace));
}

int 
main (int argc, char *argv[])
{
  for (uint32_t i = 0; i<255; i++)
        PktTxCount[i]=0;

  for (uint32_t i=0; i<100; i++)
	arrival_times[i]=0.0;

  double inter_arrival_times[100];
  double mean_irt=0;
  double variance_irt=0;
  double max_irt=0;
  double min_irt=10;

  for (uint32_t i=0; i<100; i++)
        inter_arrival_times[i]=0.0;

  //----------------------- WAVE Node Configuration ----------------------------------------------
  uint32_t packetSize = 500; //For Pkt_size = 500, a node can transmit maximum 43 packets in 50 milliseconds, with slot=16, sifs=32
  uint32_t noPackets = 100;
  uint32_t simTime = 200;
  double interval = 0.1;
  double gpsAccuracyNs = 40;  
  int nodeSpeed = 10;
  int nodePause = 0;
  bool nodeMobility = false;
  double distanceToRx = 50.0;
  double RxGain = 4.3;
  double TxGain = 10;
  double freq = 5900;
  uint32_t sifs = 32;
  uint32_t pifs = 32 + 13;
  uint32_t eifsnodifs = 32 + 88;
  uint32_t vehicle_density = 120; // in cars/km
  uint32_t road_length = 1; // in kms
  uint32_t no_lanes = 6; 
  uint32_t lane_separation = 3;
  uint32_t slot = 16;
  double Sensitivity = -91; //RxGain, TxGain and Sensitivity are such that Transmission range = Sensing range = 300m.
  double CcaSensitivity = -91;
  //----------------------- IEEE 802.11a Node Configuration ----------------------------------------------
  bool verbose = false;
  uint32_t iPacketSize = 1200; //Rate = 6 Mbps, can assume 54 Mbps with 10800 byte packet length
  uint32_t inoPackets = 100;
  double iInterval = 0.1; // seconds
  double iStartTime = 0.1; // seconds
  double iDistanceToRx = 55.0; // meters
  uint32_t ifreq = 5900; //not sure of the value yet
  double iRxGain = 4.3;
  double iTxGain = 10;  
  uint32_t isifs = 16;
  uint32_t ipifs = 25;
  uint32_t ieifsnodifs = 60;
  uint32_t islot = 1;
  double iSensitivity = -91; //iRxGain, iTxGain and iSensitivity are such that Transmission range = Sensing range = 500m.
  double iCcaSensitivity = -95; 
  uint32_t sensingRange = 0;
  //----------------------- Command Line inputs ----------------------------------------------
  CommandLine cmd;
  cmd.AddValue ("vehicle_density", "Vehicle density in number of cars/km", vehicle_density);
  cmd.AddValue ("road_length", "Length of road in kilometers", road_length);
  cmd.AddValue ("no_lanes", "Number of lanes", no_lanes);
  cmd.AddValue ("lane_separation", "Separation between lanes in m",lane_separation);
  cmd.AddValue ("nodeMobility", "Whether nodes or mobile or not", nodeMobility);
  cmd.AddValue ("packetSize", "Packet Size", packetSize);
  cmd.AddValue ("iPacketSize", "Interferer Packet Size", iPacketSize);
  cmd.AddValue ("noPackets", "Number of Packets", noPackets);
  cmd.AddValue ("inoPackets", "Interferer Number of Packets", inoPackets);
  cmd.AddValue ("interval", "Interval between packets", interval);
  cmd.AddValue ("iInterval", "Interval between interferer packets)", iInterval);
  cmd.AddValue ("gpsAccuracyNs", "gpsAccuracy in nanoseconds", gpsAccuracyNs);
  cmd.AddValue ("nodeSpeed", "node Speed in m/s", nodeSpeed);
  cmd.AddValue ("nodePause", "node pause in seconds", nodePause);
  cmd.AddValue ("iStartTime", "Interference Start Time", iStartTime);
  cmd.AddValue ("distanceToRx", "Distance between WAVE source and receiver (stationary case)", distanceToRx);
  cmd.AddValue ("iDistanceToRx", "Distance between Interferer source and receiver", iDistanceToRx);
  cmd.AddValue ("freq", "Frequency of WAVE node", freq);
  cmd.AddValue ("ifreq", "Frequency of 802.11a node", ifreq);
  cmd.AddValue ("RxGain", "Gain of Receiver", RxGain);
  cmd.AddValue ("TxGain", "Gain of Transmitter", TxGain);
  cmd.AddValue ("iRxGain", "Gain of Interfering Receiver", iRxGain);
  cmd.AddValue ("iTxGain", "Gain of Interfering Transmitter", iTxGain);
  cmd.AddValue ("verbose", "Interference Reception Verbose", verbose);
  cmd.AddValue ("sifs", "SIFS of WAVE", sifs);
  cmd.AddValue ("isifs", "SIFS of Interferer", isifs);
  cmd.AddValue ("pifs", "PIFS of WAVE", pifs);
  cmd.AddValue ("ipifs", "PIFS of Interferer", ipifs);
  cmd.AddValue ("eifsnodifs", "EIFSnoDIFS of WAVE", eifsnodifs);
  cmd.AddValue ("ieifsnodifs", "EIFSnoDIFS of Interferer", ieifsnodifs);
  cmd.AddValue ("slot", "Slot Time for WAVE", slot);
  cmd.AddValue ("islot", "Slot Time for Interferer", islot);
  cmd.AddValue ("Sensitivity", "The energy of DSRC received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.", Sensitivity);
  cmd.AddValue ("CcaSensitivity", "The energy of DSRC received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.", CcaSensitivity);
  cmd.AddValue ("iSensitivity", "The energy of Interferer received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.", iSensitivity);
  cmd.AddValue ("iCcaSensitivity", "The energy of Interferer received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.", iCcaSensitivity);
  cmd.AddValue ("seed", "Seed Value", seed);
  cmd.AddValue ("run", "Run Number", run);
  cmd.Parse (argc, argv);
  // --------------------------------------------------------------------------------------

  if (nodeMobility)
        no_nodes = vehicle_density * road_length;
  else
        no_nodes = 2;

  if (iCcaSensitivity == -91){
	sensingRange = 300;
  }
  else if (iCcaSensitivity == -95){
	sensingRange = 400;
  }
  else if (iCcaSensitivity == -98){
	sensingRange = 500;
  }
  
  ConfigStore config;
  config.ConfigureDefaults ();
  
  //---------------------------------------------------------------------------------------
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<YansWifiChannel> wirelessChannel = channel.Create ();

  //------------------------------------------Wave Traffic----------------------------------------------
  DsrcNodes example;
  example.mobility=nodeMobility;
  example.Initialize (vehicle_density, no_lanes, road_length, lane_separation, wirelessChannel, noPackets, packetSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause, distanceToRx, RxGain, TxGain, freq, sifs, pifs, eifsnodifs, slot, Sensitivity, CcaSensitivity);
  example.SendWsmpExample ();

  //------------------------------------------ Interferer Traffic--------------------------------------------
  Interferer interferer;
  interferer.mobility = false;
  interferer.iverbose = verbose;
  interferer.SendWsmpExample (wirelessChannel, inoPackets, iPacketSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause, iDistanceToRx, iRxGain, iTxGain, ifreq, isifs, ipifs, ieifsnodifs, islot, iSensitivity, iCcaSensitivity);

  config.ConfigureAttributes ();

  Simulator::Stop (Seconds (simTime));

  AnimationInterface anim ("animation.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  std::cout<<"PDR = "<<example.m_rxPacketCounter/(1.0*noPackets)<<std::endl;

  //std::cout<<"Transmitted Packets = "<<PktTxCount<<std::endl;


//  /*
  std::cout<<"\nPackets attempted to be transmitted by nodes\n";
  std::cout<<"Node\tNo_packets\n";
  for (uint32_t i=0; i<no_nodes; i++)
    std::cout<<i<<"\t"<<PktTxCount[i]<<std::endl;

	
//  */  
 std::cout<<"Printing IRT Values \n";
 std::cout<<"VD = "<<vehicle_density<<", SR = "<<sensingRange<<", IFS = "<<isifs<<std::endl;
 for (uint32_t i=0; i<100; i++)
	if (arrival_times[i] != 0){
		//std::cout<<i+1<<"\t"<<arrival_times[i]<<"\n";
		if (i==0)
			inter_arrival_times[i]=arrival_times[i];
		else
			inter_arrival_times[i]=arrival_times[i]-arrival_times[i-1];
		std::cout<<i<<"\t"<<inter_arrival_times[i]<<"\n";
	}
  
 std::cout<<"Done Printing IRT Values \n";
   
  for (uint32_t i=0; i<example.m_rxPacketCounter; i++){
        mean_irt += inter_arrival_times[i];
        if (inter_arrival_times[i] > max_irt)
                max_irt = inter_arrival_times[i];
        if (inter_arrival_times[i] < min_irt)
                min_irt = inter_arrival_times[i];
 }
 mean_irt = mean_irt/example.m_rxPacketCounter;

 for (uint32_t i=0; i<example.m_rxPacketCounter; i++)
        variance_irt += (inter_arrival_times[i] - mean_irt)*(inter_arrival_times[i] - mean_irt);
 variance_irt = variance_irt/example.m_rxPacketCounter;

 std::cout<<"Mean IRT = "<<mean_irt<<std::endl;
 std::cout<<"Variance IRT = "<<variance_irt<<std::endl;
 std::cout<<"ac Throughput = "<<26370700+9*interferer.m_rxPacketCounter*iPacketSize*8.0/10<<std::endl;
 std::cout<<"Max IRT = "<<max_irt<<std::endl;
 std::cout<<"Min IRT = "<<min_irt<<std::endl;

   
  return 0;
}

