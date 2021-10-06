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

#include "wavetable.hpp"

// https://www.boost.org/doc/libs/1_63_0/doc/html/program_options/tutorial.html
// Configures variables that can be set for program and their defaults
namespace po = boost::program_options;


/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}


/***********************************************************************
 * Main function
 **********************************************************************/

int main(int argc, char* argv[])
{
    uhd::set_thread_priority_safe();

    // Network adapters need some configuration to work with x300. This script does all that. Proved to work for GNU Radio 100 times before.
    std::cout << "Configuring network adapter settings" << std::endl;
	// NB: This file should be used to set ALL variables
    system("./usrp_n210_init.sh");

    // 
    std::string devAddress, file, ref, wave_type, pps, otw, print_time;
    size_t total_num_samps, numChannels;
    double tx_rate, rx_rate, tx_freq, rx_freq, tx_gain, rx_gain, tx_bw, rx_bw;
    double wave_freq, lo_offset, total_time, spb, setup_time;
    float ampl;
	uhd::rx_metadata_t md;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("dev", po::value<std::string>(&devAddress)->default_value("addr0=192.168.10.2"), "single uhd device address args (dev=addr0=192.168.10.2")
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
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset)->default_value(0.0),"Offset for frontend LO in Hz (optional)")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "pps source (gpsdo, internal, external)")
		("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (gpsdo, internal, external)")
		("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("print", po::value<std::string>(&print_time)->default_value("N"), "y/N")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
    ;

    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Rx multi samples to file %s") % desc << std::endl;
        std::cout << std::endl << "This application transmits and recieves data on a single ettus N210\n" << std::endl;
        return ~0;
    }

    // create a usrp device
    // single board, 2 slots on ettusN210
    //printing IP which device is recorded as using
    std::cout << boost::format("Creating the TxRx usrp device with: %s...") % devAddress
              << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(devAddress);
    std::cout << std::endl;
    // setting rx and tx subdevice
 

    //---------------------------------------------------------------------
    //              Configuring Tx an RX channels
    //---------------------------------------------------------------------

    usrp->set_tx_subdev_spec(uhd::usrp::subdev_spec_t("A:0"), 0);
    usrp->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("A:0"), 0);
    usrp->set_rx_antenna ("RX2",0);
    usrp->set_tx_antenna ("TX/RX",0);
    
    //printing to confirm ports set
    std::cout << boost::format("Tx: %s") % usrp->get_tx_antenna() << std::endl;
    std::cout << boost::format("Rx: %s") % usrp->get_rx_antenna() << std::endl;
    


    // Lock mboard clocks 
    if (vm.count("ref")) {
        usrp->set_clock_source(ref);
    }

    //printing clock for mBoard 0
    std::cout << boost::format("Clock: %s \n") % usrp->get_clock_source(0) << std::endl;

    //printing device hardware info
    std::cout << boost::format("Using Device: %s \n") % usrp->get_pp_string()
              << std::endl;

        
    //---------------------------------------------------------------------
    //              Configuring Tx an RX channels
    //---------------------------------------------------------------------


    // set the transmit sample rate
    if (not vm.count("tx-rate")) { //count instances of flag
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) { //count instances of flag
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) { //count instances of flag
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }
    //Note 0 channel transmission is hardcoded
    size_t channel = 0;
    uhd::tune_request_t tx_tune_request(tx_freq);
    if (vm.count("tx-int-n"))
        tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_tx_freq(tx_tune_request, channel);
    //printing transmit channel
    std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                << std::endl;
    std::cout << boost::format("Actual TX Freq: %f MHz...")
                        % (usrp->get_tx_freq(channel) / 1e6)
                << std::endl
                << std::endl;


    // set the rf gain
    if (vm.count("tx-gain")) {
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                    << std::endl;
        usrp->set_tx_gain(tx_gain, channel);
        std::cout << boost::format("Actual TX Gain: %f dB...")
                            % usrp->get_tx_gain(channel)
                    << std::endl
                    << std::endl;
    }

    // set the analog frontend filter bandwidth
    if (vm.count("tx-bw")) {
        std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                    << std::endl;
        usrp->set_tx_bandwidth(tx_bw, channel);
        std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                            % usrp->get_tx_bandwidth(channel)
                    << std::endl
                    << std::endl;
    }

    // set the antenna
    usrp->set_tx_antenna("TX/RX", channel);

    std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
    uhd::tune_request_t rx_tune_request(rx_freq);
    if (vm.count("rx-int-n"))
        rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_rx_freq(rx_tune_request, channel);
    std::cout << boost::format("Actual RX Freq: %f MHz...")
                        % (usrp->get_rx_freq(channel) / 1e6)
                << std::endl
                << std::endl;
    
    //---------------------------------------------------------------------
    //              Configuring Tx an RX channels
    //---------------------------------------------------------------------

    // set the receive rf gain
    if (vm.count("rx-gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                    << std::endl;
        usrp->set_rx_gain(rx_gain, channel);
        std::cout << boost::format("Actual RX Gain: %f dB...")
                            % usrp->get_rx_gain(channel)
                     << std::endl
                     << std::endl;
    }

    // set the receive analog frontend filter bandwidth
    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                << std::endl;
    usrp->set_rx_bandwidth(rx_bw, channel);
    std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                        % (usrp->get_rx_bandwidth(channel) / 1e6)
                << std::endl
                << std::endl;


    // set the receive antenna
    usrp->set_rx_antenna("RX2", channel);
    

    //---------------------------------------------------------------------
    //              Configuring Transmitted TWaveform
    //---------------------------------------------------------------------


    // for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST") {
        wave_freq = usrp->get_tx_rate() / 2;
    }

    // error when the waveform is not possible to generate
    if (std::abs(wave_freq) > usrp->get_tx_rate() / 2) {
        throw std::runtime_error("Tx wave freq out of Nyquist zone");
    }

    if (usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
        throw std::runtime_error("Tx wave freq too small for table");
    }

    // pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t step = std::lround(wave_freq / usrp->get_tx_rate() * wave_table_len);
    size_t index = 0;
    

    //---------------------------------------------------------------------
    //              Data Handling Config
    //---------------------------------------------------------------------

    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0) {
        spb = tx_stream->get_max_num_samps() * 10;
    }
    std::vector<std::complex<float>> buff(spb);
    std::vector<std::complex<float>*> buffs(channel, &buff.front());

    // pre-fill the buffer with the waveform
    for (size_t n = 0; n < buff.size(); n++) {
        buff[n] = wave_table(index += step);
    }

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;

    usrp->set_time_now(0.0);

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    // Set up metadata. We start streaming a bit in the future
    // to allow MIMO operation:
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec      = usrp->get_time_now() + uhd::time_spec_t(0.1);
    
    return 0;
}