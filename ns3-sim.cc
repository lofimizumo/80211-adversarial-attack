#include "ns3/applications-module.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/double.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/gnuplot.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/qos-utils.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/txop.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-state.h"
#include "ns3/wifi-psdu.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ns3-ai-module.h"

#include <fstream>
#include <string>
#include <sys/stat.h> // for mkdir
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RateAdaptationDistance");
struct AiConstantRateEnv
{
  uint8_t mcs;
  uint8_t max_mcs;
  double time;
  double throughput;
  double snr;
} Packed;

struct AiConstantRateAct
{
  bool shouldJam;
} Packed;
Ns3AIRL<AiConstantRateEnv, AiConstantRateAct> * m_ns3ai_mod;

std::ofstream g_csvFile;
Time timestampDIFS;
Time currentTime;
Time slotTime = MicroSeconds(9); 

double totalBytesReceived = 0.0;
Time lastTime = Seconds(0);
Ptr<Node> wifiIAP1; 
double thresholdThroughput = 2e7; 
double lower_thresholdThroughput = 1e7; 
double g_current_tpt = 0.0;
u_int32_t attack_frame_count = 0;
u_int32_t total_packet_count = 0;
u_int32_t packet_received_count = 0;
bool aiJammingMode = true;

void InitializeModule() {
    uint16_t id = 2337;
    m_ns3ai_mod = new Ns3AIRL<AiConstantRateEnv, AiConstantRateAct>(id);
    m_ns3ai_mod->SetCond (2, 0);
    std::cout<<"Ns3AIRL initialized"<<std::endl;
}

void
InitStream(int run)
{
    std::string dirName = "ReinRate-scenario5";

    mkdir(dirName.c_str(), 0777); 

    std::string fileName = dirName + "/scene5_Minstrel_run_" + std::to_string(run) + ".csv";
    g_csvFile.open(fileName);

    if (g_csvFile.is_open())
    {
        g_csvFile << "Time,Throughput (Mbps)" << std::endl; // CSV header
    }
    else
    {
        std::cerr << "Failed to open file: " << fileName << std::endl;
    }
}

void
PhyTxTrace(std::string context, Ptr<const Packet> p)
{
    WifiMacHeader macHeader;
    Ptr<Packet> copyPacket = p->Copy();
    copyPacket->RemoveHeader(macHeader);
    std::cout << "Packet sent to: " << macHeader.GetAddr1() << std::endl;
}

void
EndSimulation()
{
    g_csvFile.close();
}

class PacketMonitor
{
  public:
    void Sniffer(std::string context, Ptr<const Packet> packet, double txPowerW);
    void TraceMCS(std::string context, const Ptr< const Packet > packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, uint16_t staId);
    void RxDrop(std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason);
    void RxEnd(std::string context, Ptr<const Packet> packet);
    void UpdateThroughput();
    void OutputCsv();

  private:
    double m_lastArrivalTime{0.0};
    std::vector<double> m_interArrivalTimes;
    std::deque<std::pair<Time, uint32_t>> m_dataPoints; // Pair of timestamp and bytes received
    std::vector<std::pair<Time, double>> m_time_tpt; // for csv output 
    std::vector<std::pair<Time, double>> m_jamming_record; // for csv output 
    double m_throughput; // Throughput in Mbps

};

void
SendPacket(Ptr<WifiMac> mac)

{
    PointerValue ptr;
    mac->GetAttribute("BE_Txop", ptr);
    Ptr<QosTxop> edca;
    edca = ptr.Get<QosTxop>();
    // std::cout << "Backoff Slots (Victim): " << edca->GetCw(0) << std::endl;
    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet>(10); // 1000 bytes dummy packet
    ns3::Mac48Address("00:00:00:00:00:01");
    ns3::Mac48Address destAddress = ns3::Mac48Address("00:00:00:00:00:01");
    mac->Enqueue(packet, destAddress);
    Simulator::Schedule(Seconds(0.001), &SendPacket, mac);
};
std::vector<std::pair<Time, double>> m_jam_times; // for csv output
void
StartJamming(Ptr<WifiMac> jammerMac, Ptr<WifiPhy> jammerPhy)
{
    if (packet_received_count < 1)
    {
        return;
    }
    attack_frame_count++;
    
    m_jam_times.push_back(std::make_pair(Simulator::Now(), g_current_tpt));

    PointerValue ptr;
    jammerMac->GetAttribute("BE_Txop", ptr);
    Ptr<QosTxop> edca;
    edca = ptr.Get<QosTxop>();
    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet>(100); // 1000 bytes dummy packet
    ns3::WifiMacHeader header;
    header.SetAddr1(ns3::Mac48Address("ff:ff:ff:ff:ff:ff"));
    ns3::Mac48Address destAddress = header.GetAddr1();
    jammerMac->Enqueue(packet, destAddress);
    packet_received_count = 0;
}

void
PacketMonitor::OutputCsv()
{
    std::ofstream csv;
    csv.open("tpt.csv", std::ios::out);
    csv << "Time,Throughput (Mbps)" << std::endl;
    for (const auto& dataPoint : m_time_tpt)
    {
        csv << dataPoint.first.GetSeconds() << "," << dataPoint.second << std::endl;
    }
    csv.close();
    std::ofstream csv_jam;
    csv_jam.open("jam.csv", std::ios::out);
    csv_jam << "Time,Throughput (Mbps)" << std::endl;
    for (const auto& dataPoint : m_jam_times)
    {
        csv_jam << dataPoint.first.GetSeconds() << "," << dataPoint.second << std::endl;
    }
    csv_jam.close();

}

void
PacketMonitor::RxDrop(std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason)
{
    // std::cout << "Packet dropped at: " << Simulator::Now().GetSeconds() << "Reason: " << reason
            //   << std::endl;
}

void
PacketMonitor::RxEnd(std::string context, Ptr<const Packet> packet)
{
    // std::cout << "Packet received at: " << Simulator::Now().GetSeconds() << "Size: "
    //           << packet->GetSize() << std::endl;
}

void PacketMonitor::UpdateThroughput()
{
    Time now = Simulator::Now();
    Time windowSize = MilliSeconds(1000); // Fixed window size of 1 second

    while (!m_dataPoints.empty() && now - m_dataPoints.front().first > windowSize)
    {
        m_dataPoints.pop_front();
    }

    uint32_t totalBytes = 0;
    for (const auto& dataPoint : m_dataPoints)
    {
        totalBytes += dataPoint.second;
    }

    if (m_dataPoints.empty())
    {
        m_throughput = 0;
    }
    else
    {
        m_throughput = (totalBytes * 8.0) / (windowSize.GetSeconds() * 1e6); // Throughput in Mbps
    }
    
    g_current_tpt = m_throughput;
    m_time_tpt.push_back(std::make_pair(now, m_throughput));
    std::cout << "Current Throughput: " << m_throughput << " Mbps" << std::endl;
}


void
PacketMonitor::TraceMCS(std::string context, const Ptr< const Packet > packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, uint16_t staId)
{
   WifiMode mode = txVector.GetMode();

    
}

void
PacketMonitor::Sniffer(std::string context, Ptr<const Packet> packet, double txPowerW)
{
    total_packet_count++;
    packet_received_count++;
    WifiMacHeader macHeader;
    Ptr<Packet> copyPacket = packet->Copy();
    if (copyPacket->PeekHeader(macHeader))
    {
        // std::cout<<"From:"<<macHeader.GetAddr2()<<" To:"<<macHeader.GetAddr1()<<"Is Ack:"<<macHeader.IsAck()<<"time:"<<Simulator::Now().GetSeconds()<<std::endl;
        // Check if the header is for a frame that sets the NAV (e.g., RTS, CTS, etc.)
        if (macHeader.IsRts() || macHeader.IsCts() || macHeader.IsData())
        {
            Time navDuration = macHeader.GetDuration();
      // std::cout << "NAV Duration: " << navDuration.GetMicroSeconds() << " microseconds" <<
            // std::endl;
        }
    }   

    Ptr<Node> attackerNode = NodeList::GetNode(1);
    Ptr<WifiNetDevice> attackerDevice = attackerNode->GetDevice(0)->GetObject<ns3::WifiNetDevice>();
    Ptr<WifiMac> attackerMac = attackerDevice->GetMac();
    // std::cout<<"attackerMac: "<<attackerMac<<std::endl;

    Ptr<WifiPhy> attackerPhy = attackerDevice->GetPhy();
    totalBytesReceived += packet->GetSize();
    currentTime = Simulator::Now();
    if (currentTime.GetSeconds() - lastTime.GetSeconds() > 0.1)
    {
        double currentThroughput =
            ((totalBytesReceived * 8) / (currentTime.GetSeconds() - lastTime.GetSeconds())) /
            1e6; // Throughput in Mbps
        totalBytesReceived = 0;
        lastTime = currentTime;
    }
    if (macHeader.IsAck())
    {
        Time now = Simulator::Now();
        m_dataPoints.push_back(std::make_pair(now, 1420));

        UpdateThroughput();
        auto env = m_ns3ai_mod->EnvSetterCond ();
        env->throughput = g_current_tpt;
        
        env->time = now.GetSeconds();
        m_ns3ai_mod->SetCompleted ();
        auto act = m_ns3ai_mod->ActionGetterCond ();
        bool shouldJam = act->shouldJam;
        m_ns3ai_mod->GetCompleted ();
        if (aiJammingMode)
        {
            if (shouldJam)
            {
                Simulator::Schedule(Seconds(0), &StartJamming, attackerMac, attackerPhy);
            }
        }
        else
        {
            if (g_current_tpt > thresholdThroughput / 1e6)
            {
                    std::cout<<"Current Throughput: "<<g_current_tpt<<std::endl;
                    std::cout<<"From:"<<macHeader.GetAddr2()<<" To:"<<macHeader.GetAddr1()<<"Is Ack:"<<macHeader.IsAck()<<"time:"<<Simulator::Now().GetSeconds()<<std::endl;
                    Simulator::Schedule(Seconds(0), &StartJamming, attackerMac, attackerPhy);
            }
        }
    }
}
/** Node statistics */
class NodeStatistics
{
  public:
    /**
     * Constructor
     * \param aps AP devices
     * \param stas STA devices
     */
    NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

    /**
     * RX callback
     * \param path path
     * \param packet received packet
     * \param from sender
     */
    void RxCallback(std::string path, Ptr<const Packet> packet, const Address& from);
    /**
     * Set node position
     * \param node the node
     * \param position the position
     */
    void SetPosition(Ptr<Node> node, Vector position);
    /**
     * Advance node position
     * \param node the node
     * \param stepsSize the size of a step
     * \param stepsTime the time interval between steps
     */
    void AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime);
    /**
     * Get node position
     * \param node the node
     * \return the position
     */
    
    void RandomMoving(Ptr<Node> node, int stepsSize, int stepsTime);
    Vector GetPosition(Ptr<Node> node);
    /**
     * \return the gnuplot 2d dataset
     */
    Gnuplot2dDataset GetDatafile();

  private:
    uint32_t m_bytesTotal;     //!< total bytes
    Gnuplot2dDataset m_output; //!< gnuplot 2d dataset
};

NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
    m_bytesTotal = 0;
}

void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address& from)
{
    // std::cout<<"Address: "<<from<<std::endl;
    m_bytesTotal += packet->GetSize();
    // std::cout<<"Total bytes: "<<m_bytesTotal<<std::endl;
}

void
NodeStatistics::SetPosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    mobility->SetPosition(position);
}

Vector
NodeStatistics::GetPosition(Ptr<Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    return mobility->GetPosition();
}

void
NodeStatistics::AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime)
{
    Vector pos = GetPosition(node);
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
    m_bytesTotal = 0;
    m_output.Add(pos.x, mbs);
    pos.x += 2*stepsSize;
    std::cout<<"pos.x: "<<pos.x<<std::endl;
    SetPosition(node, pos);
    Simulator::Schedule(Seconds(stepsTime),
                        &NodeStatistics::AdvancePosition,
                        this,
                        node,
                        stepsSize,
                        stepsTime);
}

void NodeStatistics::RandomMoving(Ptr<Node> node, int stepsSize, int stepsTime)
{
    srand(2222);

    Vector pos = GetPosition(node);
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
    m_bytesTotal = 0;
    m_output.Add(pos.x, mbs);

    int randomStep = (std::rand() % (stepsSize * 2 + 1)) - stepsSize; 

    pos.x += 5*randomStep;
    std::cout<<"pos.x: "<<pos.x<<std::endl;

    SetPosition(node, pos);
    Simulator::Schedule(Seconds(stepsTime),
                        &NodeStatistics::RandomMoving,
                        this,
                        node,
                        stepsSize,
                        stepsTime);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile()
{
    return m_output;
}

/**
 * Callback for 'Rate' trace source
 *
 * \param oldRate old MCS rate (bits/sec)
 * \param newRate new MCS rate (bits/sec)
 */
void
RateCallback(uint64_t oldRate, uint64_t newRate)
{
    NS_LOG_INFO("Rate " << newRate / 1000000.0 << " Mbps");
}

int
main(int argc, char* argv[])
{
    uint32_t rtsThreshold = 65535;
    std::string staManager = "ns3::MinstrelHtWifiManager";
    std::string apManager = "ns3::IdealWifiManager";
    std::string standard = "802.11n-5GHz";
    std::string outputFileName = "minstrelHT";
    uint32_t BeMaxAmpduSize = 0;
    bool shortGuardInterval = false;
    uint32_t chWidth = 20;
    int ap1_x = 0;
    int ap1_y = 0;
    int ap2_x = 0;
    int ap2_y = 0;
    int sta1_x = 5;
    int sta1_y = 0;
    int steps = 40;
    int stepsSize = 1;
    int stepsTime = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("staManager", "Rate adaptation manager of the STA", staManager);
    cmd.AddValue("apManager", "Rate adaptation manager of the AP", apManager);
    cmd.AddValue("standard", "Wifi standard (a/b/g/n/ac only)", standard);
    cmd.AddValue("shortGuardInterval",
                 "Enable Short Guard Interval in all stations",
                 shortGuardInterval);
    cmd.AddValue("channelWidth", "Channel width of all the stations", chWidth);
    cmd.AddValue("rtsThreshold", "RTS threshold", rtsThreshold);
    cmd.AddValue("BeMaxAmpduSize", "BE Mac A-MPDU size", BeMaxAmpduSize);
    cmd.AddValue("outputFileName", "Output filename", outputFileName);
    cmd.AddValue("steps", "How many different distances to try", steps);
    cmd.AddValue("stepsTime", "Time on each step", stepsTime);
    cmd.AddValue("stepsSize", "Distance between steps", stepsSize);
    cmd.AddValue("AP1_x", "Position of AP1 in x coordinate", ap1_x);
    cmd.AddValue("AP1_y", "Position of AP1 in y coordinate", ap1_y);
    cmd.AddValue("STA1_x", "Position of STA1 in x coordinate", sta1_x);
    cmd.AddValue("STA1_y", "Position of STA1 in y coordinate", sta1_y);
    cmd.Parse(argc, argv);

    InitializeModule();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    int simuTime = steps * stepsTime;

    if (standard != "802.11a" && standard != "802.11b" && standard != "802.11g" &&
        standard == "802.11n-2.4GHz" && standard != "802.11n-5GHz" && standard != "802.11ac")
    {
        NS_FATAL_ERROR("Standard " << standard << " is not supported by this program");
    }

    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(1);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    std::string frequencyBand;
    if (standard == "802.11b" || standard == "802.11g" || standard == "802.11n-2.4GHz")
    {
        frequencyBand = "BAND_2_4GHZ";
    }
    else
    {
        frequencyBand = "BAND_5GHZ";
    }
    wifiPhy.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(chWidth) + ", " + frequencyBand + ", 0}"));

    wifiPhy.Set("CcaSensitivity", DoubleValue(-110));
    wifiPhy.Set("RxNoiseFigure", DoubleValue(0));
    wifiPhy.DisablePreambleDetectionModel();

    NetDeviceContainer wifiApDevices;
    NetDeviceContainer wifiStaDevices;
    NetDeviceContainer wifiDevices;

    WifiHelper wifi;


    wifi.SetStandard(WIFI_STANDARD_80211n);


    WifiMacHelper wifiMac;

    wifi.SetRemoteStationManager(staManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

    Ssid ssid = Ssid("AP");
    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes.Get(0)));

    wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

    ssid = Ssid("AP");
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(0)));
    wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(1)));

    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                UintegerValue(BeMaxAmpduSize));

    wifiDevices.Add(wifiStaDevices);
    wifiDevices.Add(wifiApDevices);

    // Set guard interval
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        BooleanValue(shortGuardInterval));

    // Configure the mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // Initial position of AP and STA
    positionAlloc->Add(Vector(ap1_x, ap1_y, 0.0));
    positionAlloc->Add(Vector(ap2_x, ap2_y, 0.0));
    positionAlloc->Add(Vector(sta1_x, sta1_y, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes.Get(0));
    mobility.Install(wifiApNodes.Get(1));
    mobility.Install(wifiStaNodes.Get(0));

    NodeStatistics atpCounter = NodeStatistics(wifiApDevices, wifiStaDevices);
    PacketMonitor monitor;
    // atpCounter.AdvancePosition(wifiStaNodes.Get(0), stepsSize, stepsTime);
    atpCounter.RandomMoving(wifiStaNodes.Get(0), stepsSize, stepsTime);

    // Configure the IP stack
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = address.Assign(wifiDevices);
    Ipv4Address sinkAddress = i.GetAddress(0);
    uint16_t port = 9;

    // Configure the CBR generator
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    ApplicationContainer apps_sink = sink.Install(wifiStaNodes.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    onoff.SetConstantRate(DataRate("60Mb/s"), 1420);
    onoff.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    onoff.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    ApplicationContainer apps_source = onoff.Install(wifiApNodes.Get(0));

    apps_sink.Start(Seconds(0.5));
    apps_sink.Stop(Seconds(simuTime));

    std::cout << "Node Id of wifiApNode1 is: " << wifiApNodes.Get(0)->GetId() << std::endl;
    std::cout << "Node Id of wifiApNode2 is: " << wifiApNodes.Get(1)->GetId() << std::endl;
    std::cout << "Node Id of wifiStaNode1 is: " << wifiStaNodes.Get(0)->GetId() << std::endl;
    std::cout << "AP1 Mac address is: " << wifiApDevices.Get(0)->GetAddress() << std::endl;
    std::cout << "AP2 Mac address is: " << wifiApDevices.Get(1)->GetAddress() << std::endl;
    std::cout << "Sta1 Mac address is: " << wifiStaDevices.Get(0)->GetAddress() << std::endl;


    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                    MakeCallback(&NodeStatistics::RxCallback, &atpCounter));

    Config::ConnectWithoutContextFailSafe(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + apManager + "/Rate",
        MakeCallback(RateCallback));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
            MakeCallback(&PacketMonitor::Sniffer, &monitor));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferTx",
            MakeCallback(&PacketMonitor::TraceMCS, &monitor));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                    MakeCallback(&PacketMonitor::RxDrop, &monitor));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",
                    MakeCallback(&PacketMonitor::RxEnd, &monitor));
    Simulator::Stop(Seconds(simuTime));
    Simulator::Run();
    monitor.OutputCsv();
    Simulator::Destroy();

    return 0;
}