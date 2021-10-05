#include <iostream>
#include <vector>
#include <complex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <ctime>

#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/utils/thread.hpp>


// https://www.boost.org/doc/libs/1_63_0/doc/html/program_options/tutorial.html
// Configures variables that can be set for program and their defaults
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    uhd::set_thread_priority_safe();

    // Network adapters need some configuration to work with x300. This script does all that. Proved to work for GNU Radio 100 times before.
    std::cout << "Configuring network adapter settings" << std::endl;
	// NB: This file should be used to set ALL variables
    system("./usrp_n210_init.sh");

    // 
    std::string devAddress, file, ref, pps, print_time;
    size_t total_num_samps, numChannels;
    double tx_rate, rx_rate, tx_freq, rx_freq, tx_gain, rx_gain, tx_bw, rx_bw;
    double bw, total_time, spb, setup_time;
	uhd::rx_metadata_t md;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("dev", po::value<std::string>(&devAddress)->default_value("addr0=192.168.10.2"), "single uhd device address args (dev=addr0=192.168.40.2, addr1=192.168.50.2)")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.bin"), "name of the file to write binary samples to")
        ("nsamps", po::value<size_t>(&total_num_samps), "total number of samples to receive")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")
        ("spb", po::value<double>(&spb)->default_value(1), "buffer multiplier") //buffer per channel
        ("tx-rate", po::value<double>(&tx_rate), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate), "rate of receive incoming samples")
        ("tx-freq", po::value<double>(&tx_freq), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq), "receive RF center frequency in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(0), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(0), "gain for the receive RF chain")
        ("tx-bw", po::value<double>(&tx_bw)->default_value(0.0), "analog frontend filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw)->default_value(0.0), "analog frontend filter bandwidth in Hz")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "pps source (gpsdo, internal, external)")
		("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (gpsdo, internal, external)")
		("print", po::value<std::string>(&print_time)->default_value("N"), "y/N")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
    ;

    //storing manual inputs
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Rx multi samples to file %s") % desc << std::endl;
        std::cout << std::endl << "This application transmits and recieves data on a single ettus N210\n" << std::endl;
        return ~0;
    }

    //git commit -m "changed to independant rx,tx freq,rate, gain"
    return 0;
}