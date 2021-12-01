//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "wavetable.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/usrp_clock/multi_usrp_clock.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/types/clock_config.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <csignal>
#include <fstream>
#include <time.h>
#include <functional>
#include <iostream>
#include <chrono>

#include <ctime>


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
 * Utilities
 **********************************************************************/
//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename(
    const std::string& base_fn, size_t n_names, size_t this_name)
{
    if (n_names == 1) {
        return base_fn;
    }

    boost::filesystem::path base_fn_fp(base_fn);
    base_fn_fp.replace_extension(boost::filesystem::path(
        str(boost::format("%02d%s") % this_name % base_fn_fp.extension().string())));
    return base_fn_fp.string();
}

/***********************************************************************
 * Read from File
 **********************************************************************/

template <typename samp_type>
void send_from_file(
    //std::vector<std::complex<samp_type>> buff,
    uhd::tx_streamer::sptr tx_streamer,
    const std::string& file, 
    size_t samps_per_buff,
    bool repeat,
    double delay,
    uhd::usrp::multi_usrp::sptr tx_usrp 
    )
{   


    do {
        
        // loop until the entire file has been read

        //std::cout << boost::format("Reading from file : %s...") %file << std::endl;
        std::ifstream infile(file.c_str(), std::ifstream::binary);
        //check to read file again
        
        uhd::tx_metadata_t md;
        md.start_of_burst = false;
        md.end_of_burst   = false;
        std::vector<samp_type> buff(samps_per_buff);  
        
        //loop through file once
        while (not md.end_of_burst and not stop_signal_called) {
            infile.read((char*)&buff.front(), buff.size() * sizeof(samp_type));
            size_t num_tx_samps = size_t(infile.gcount() / sizeof(samp_type));

            md.end_of_burst = infile.eof();

            const size_t samples_sent = tx_streamer->send(&buff.front(), num_tx_samps, md);
            if (samples_sent != num_tx_samps) {
                UHD_LOG_ERROR("TX-STREAM",
                    "The tx_stream timed out sending " << num_tx_samps << " samples ("
                                                    << samples_sent << " sent).");
                return;
            }
        }

        infile.close();
        
        if (repeat and delay > 0.0) {
            
            //insert delay from signal itself

            // double a = 0;
            // bool wait = true;
            // uhd::time_spec_t tv_start;
            // uhd::time_spec_t tv_end;

            // tv_start = tx_usrp->get_time_now();
            // a = tv_start.get_tick_count(1e+9);
    
            // while (wait)
            // {
            //     tv_end = tx_usrp->get_time_now();
            //     //float time = (tv_end.tv_sec + tv_end.tv_nsec/1e+9) - (tv_start.tv_sec + tv-start.tv_nsec/1e+9);

            //     float time = ((tv_end.get_tick_count(1e+9)) - a)*1/1e+9;

            //     if (time > delay)
            //     {
            //         wait  = false;
            //     }
                
               
            // }   
    
            
            //std::this_thread::sleep_for(std::chrono::nanoseconds(int64_t(delay * 1000000000)));
            // std::cout << boost::format(" %f s") % ( tv.tv_nsec)
            // << std::endl;
        }
        
    }while(repeat and not stop_signal_called);

    
}


/***********************************************************************
 * recv_to_file function
 **********************************************************************/
template <typename samp_type>
void recv_to_file(uhd::usrp::multi_usrp::sptr usrp,
    const std::string& cpu_format,
    const std::string& wire_format,
    const std::string& file,
    size_t samps_per_buff,
    int num_requested_samples,
    double settling_time,
    std::vector<size_t> rx_channel_nums)
{
    int num_total_samps = 0;
    // create a receive streamer
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    stream_args.channels             = rx_channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Prepare buffers for received samples and metadata
    uhd::rx_metadata_t md;
    std::vector<std::vector<samp_type>> buffs(
        rx_channel_nums.size(), std::vector<samp_type>(samps_per_buff));
    // create a vector of pointers to point to each of the channel buffers
    std::vector<samp_type*> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++) {
        buff_ptrs.push_back(&buffs[i].front());
    }

    // Create one ofstream object per channel
    // (use shared_ptr because ofstream is non-copyable)
    std::vector<std::shared_ptr<std::ofstream>> outfiles;
    for (size_t i = 0; i < buffs.size(); i++) {
        const std::string this_filename = generate_out_filename(file, buffs.size(), i);
        outfiles.push_back(std::shared_ptr<std::ofstream>(
            new std::ofstream(this_filename.c_str(), std::ofstream::binary)));
    }
    UHD_ASSERT_THROW(outfiles.size() == buffs.size());
    UHD_ASSERT_THROW(buffs.size() == rx_channel_nums.size());
    bool overflow_message = true;
    double timeout =
        settling_time + 0.1f; // expected settling time + padding for first recv

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd);

    while (not stop_signal_called
           and (num_requested_samples > num_total_samps or num_requested_samples == 0)) {
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samps_per_buff, md, timeout);
        timeout             = 0.1f; // small timeout for subsequent recv
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr
                    << boost::format(
                           "Got an overflow indication. Please consider the following:\n"
                           "  Your write medium must sustain a rate of %fMB/s.\n"
                           "  Dropped samples will not be written to the file.\n"
                           "  Please modify this example for your purposes.\n"
                           "  This message will not appear again.\n")
                           % (usrp->get_rx_rate() * sizeof(samp_type) / 1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error(
                str(boost::format("Receiver error %s") % md.strerror()));
        }

        num_total_samps += num_rx_samps;

        for (size_t i = 0; i < outfiles.size(); i++) {
            outfiles[i]->write(
                (const char*)buff_ptrs[i], num_rx_samps * sizeof(samp_type));
        }
    }

    // Shut down receiver
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    // Close files
    for (size_t i = 0; i < outfiles.size(); i++) {
        outfiles[i]->close();
    }
}


/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // transmit variables to be set by po
    std::string tx_args, wave_type, tx_ant, tx_subdev, otw, tx_channels;
    double tx_rate, tx_freq, tx_gain, wave_freq, tx_bw;
    float ampl;

    // receive variables to be set by po
    std::string rx_args, file_rx, file_tx, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, spb;
    double rx_rate, rx_freq, rx_gain, rx_bw;
    double settling, delay;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value(""), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value(""), "uhd receive device address args")
        ("file-tx", po::value<std::string>(&file_tx), "name of the file to read binary samples from")
        ("file-write", po::value<std::string>(&file_rx)->default_value("rx.dat"), "name of the file to write binary to")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("tx-rate", po::value<double>(&tx_rate), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate), "rate of receive incoming samples")
        ("tx-freq", po::value<double>(&tx_freq), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq), "receive RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("tx-gain", po::value<double>(&tx_gain), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain), "gain for the receive RF chain")
        ("tx-ant", po::value<std::string>(&tx_ant), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev), "receive subdevice specification")
        ("tx-bw", po::value<double>(&tx_bw), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw), "analog receive filter bandwidth in Hz")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("CONST"), "waveform type (CONST, SQUARE, RAMP, SINE) - file-tx takes precedence")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(0), "waveform frequency in Hz")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode(sc8 or sc16)")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"A:0\" only")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"B:0\" only")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
        ("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (in seconds)")
        ("repeat", "repeatedly transmit file")
       
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD TXRX Loopback to File %s") % desc << std::endl;
        return ~0;
    }

    bool repeat = vm.count("repeat") > 0;

    //selecting which board in slave
    size_t master_index = 0;
    size_t slave_index = 1;


    uhd::device_addr_t dev_addr;

    //unsure how to properly configure addr0/1
    dev_addr["addr0"] = tx_args;
    dev_addr["addr1"] = rx_args;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr); //makig a multi usrp for rx and tx

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the transmit usrp sub device with: %s...") % tx_args
              << std::endl;
    usrp->set_tx_subdev_spec(tx_channels, master_index);
    std::cout << std::endl;
    std::cout << boost::format("Creating the receive usrp sub device with: %s...") % rx_args
              << std::endl;
    usrp->set_rx_subdev_spec(rx_channels, slave_index);

    //starting time synchronisation
    std::cout << boost::format("\nTime Synchronisation") << std::endl;

    //time sync code
    uhd::time_spec_t ts = usrp->get_time_now(slave_index);
    uhd::time_spec_t tm = usrp->get_time_now(master_index);

    std::cout << boost::format("Board time: %f (not synced)") % tm.get_frac_secs()
              << std::endl;
    std::cout << boost::format("slave time: %f (not synced)") % ts.get_frac_secs()
    << std::endl;

    

    std::cout << boost::format("Configuring UHD - %f - as slave") % slave_index
              << std::endl;

    usrp->set_clock_source("internal", master_index);

    //should automatically sync with master after this line   
    usrp->set_time_source("mimo", slave_index);
    usrp->set_clock_source("mimo", slave_index);

    std::this_thread::sleep_for (std::chrono::milliseconds(50));
    
    //checking new times
    ts = usrp->get_time_now(slave_index);
    tm = usrp->get_time_now(master_index); // channel 0 - master - tx
    
    //Time shouold be synced at this point
    std::cout << boost::format("master time %s source with time: %f (synched)") %usrp->get_time_source(master_index) % tm.get_frac_secs()
              << std::endl;
    std::cout << boost::format("slave time %s source with time: %f (synched)") %usrp->get_time_source(slave_index) % ts.get_frac_secs()
    << std::endl;

    std::cout << boost::format("\n") 
    << std::endl;

    std::cout << boost::format("Channel: Usrp number ")
               << std::endl;

    std::cout << boost::format("Using Devices: %s") % usrp->get_pp_string()
               << std::endl;


    /****************************
    * Sample Params
    *****************************/

    std::cerr << "Rates common across all TX channels and RX channels (but not the necessarily the same between TX and RX) \n"
                  << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    usrp->set_tx_rate(tx_rate,0);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    usrp->set_rx_rate(rx_rate,0);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (usrp->get_rx_rate(0) / 1e6)
              << std::endl
              << std::endl;

     /****************************
    * TX Params
    *****************************/

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    
    //Tx channel Config
        std::cout << "Configuring TX Channel " << tx_channels << std::endl;
        
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                    << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_tx_freq(tx_tune_request, 0);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                            % (usrp->get_tx_freq(0) / 1e6)
                    << std::endl
                    << std::endl;

        // set the rf gain
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                      << std::endl;
            usrp->set_tx_gain(tx_gain, master_index);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % usrp->get_tx_gain(0)
                      << std::endl
                      << std::endl;
        }

        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % (tx_bw/1e6)
                      << std::endl;
            usrp->set_tx_bandwidth(tx_bw, master_index);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % (usrp->get_tx_bandwidth(0)/1e6)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            usrp->set_tx_antenna(tx_ant, master_index);
    

    /****************************
    * RX Params
    *****************************/

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(rx_tune_request, 0);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (usrp->get_rx_freq(0) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                      << std::endl;
            usrp->set_rx_gain(rx_gain, 0);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % usrp->get_rx_gain(0)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            usrp->set_rx_bandwidth(rx_bw, 0);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (usrp->get_rx_bandwidth(0) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            usrp->set_rx_antenna(rx_ant, 0);
    


    /****************************
    * Local Oscillators
    *****************************/

   // 2021/11/16
    //this may be an issue: need to figure out if local oscillator needs to be the same?

    // Check Ref and LO Lock detect
    std::vector<std::string> tx_sensor_names, rx_sensor_names;
    tx_sensor_names = usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
     }
    rx_sensor_names = usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    /****************************
    * Comms/Timing Params
    *****************************/

    //this is already done above?

    tx_sensor_names = usrp->get_mboard_sensor_names(master_index);
     if (1)
//         and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
//                 != tx_sensor_names.end())) 
    {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", master_index);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
     }
//     if ((ref == "external")
//         and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
//                 != tx_sensor_names.end())) {
//         uhd::sensor_value_t ref_locked = tx_usrp->get_mboard_sensor("ref_locked", 0);
//         std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
//                   << std::endl;
//         UHD_ASSERT_THROW(ref_locked.to_bool());
//     }

    rx_sensor_names = usrp->get_mboard_sensor_names(slave_index);
     if (1)
//         and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
//                 != rx_sensor_names.end())) 
    {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", slave_index);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
     }
//     if ((ref == "external")
//         and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
//                 != rx_sensor_names.end())) {
//         uhd::sensor_value_t ref_locked = rx_usrp->get_mboard_sensor("ref_locked", 0);
//         std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
//                   << std::endl;
//         UHD_ASSERT_THROW(ref_locked.to_bool());
//     }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));
  
   /****************************
    * TX Thread
    *****************************/

    // create a transmit streamer
    std::string cpu_format;
    std::vector<size_t> channel_nums;
    if (type == "double")
        cpu_format = "fc64";
    else if (type == "float")
        cpu_format = "fc32";
    else if (type == "short")
        cpu_format = "sc16";
    uhd::stream_args_t stream_args(cpu_format, otw);
    //channel_nums.push_back(boost::lexical_cast<size_t>(channel));
    //stream_args.channels             = ;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0)
        spb = tx_stream->get_max_num_samps() * 10;
    int num_channels = 1;  

    // send from file
    // start transmit worker thread
    boost::thread_group transmit_thread;

    if (type == "double"){
        transmit_thread.create_thread(std::bind(
        &send_from_file<std::complex<double>>, tx_stream, file_tx,spb, repeat, delay,usrp));
    }
    else if (type == "float"){
        transmit_thread.create_thread(std::bind(
        &send_from_file<std::complex<float>>, tx_stream,file_tx,spb,repeat, delay,usrp));
    }
    else if (type == "short"){
        transmit_thread.create_thread(std::bind(
        &send_from_file<std::complex<short>>, tx_stream, file_tx,spb,repeat, delay,usrp));
    }
    else
        throw std::runtime_error("Unknown type " + type);

    /****************************
    * RX Thread
    *****************************/

    std::vector<size_t> tmp_channels;
    tmp_channels.push_back(0);

    //recv to file
    
    if (type == "double")
        recv_to_file<std::complex<double>>(
            usrp, "fc64", otw, file_rx, spb, total_num_samps, settling, tmp_channels);
    else if (type == "float")
        recv_to_file<std::complex<float>>(
            usrp, "fc32", otw, file_rx, spb, total_num_samps, settling, tmp_channels);
    else if (type == "short")
        recv_to_file<std::complex<short>>(
            usrp, "sc16", otw, file_rx, spb, total_num_samps, settling, tmp_channels);
    else {
        // clean up transmit worker
        stop_signal_called = true;
        transmit_thread.join_all();
        throw std::runtime_error("Unknown type " + type);
    }

    while (not stop_signal_called) {
        
         std::this_thread::sleep_for (std::chrono::seconds(1));
    }

    
    // clean up transmit worker
    stop_signal_called = true;
    transmit_thread.join_all();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
