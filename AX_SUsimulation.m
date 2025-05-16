%%Reference: 
%https://www.mathworks.com/help/wlan/ug/recovery-procedure-for-an-802-11ax-packet.html
%https://www.mathworks.com/help/wlan/ug/802-11ax-packet-error-rate-simulation-for-single-user-format.html

function throughput  = AX_SUsimulation(cfgSU, TGAXchannel, snr, Numpacket, PL_db)
   
    OFDMInfo = wlanHEOFDMInfo ('HE-Data',cfgSU);
    IPilot = OFDMInfo.PilotIndices; %Indices of pilot within the active subcarriers
    IData  = OFDMInfo.DataIndices; %Indices of data within the active subcarriers
    ind = wlanFieldIndices(cfgSU);
    numSNR = length(snr);
    throughput = zeros(numSNR, 1);
    bandwidth = cfgSU.ChannelBandwidth;
    samplerate = wlanSampleRate(cfgSU);
    txTime = transmitTime(cfgSU);
    cfgLength = getPSDULength(cfgSU)*8;
    apepBits = cfgSU.APEPLength * 8;
    trailing = 100;
    pfOffset = comm.PhaseFrequencyOffset('SampleRate', samplerate, 'FrequencyOffsetSource', 'Input port');

    %simulate every SNR case
    parfor i = 1 : length(snr)
        error = 0;
        ipkt = 0;
        %simulate Numpacket times
        while ipkt <= Numpacket && error <= Numpacket / 5
            reset(TGAXchannel);
            ipkt  = ipkt + 1;

            %generate random psdu and corresponding waveform
            PSDU_TX = randi([0 1], cfgLength, 1);
            tx_wave = wlanWaveformGenerator(PSDU_TX,cfgSU);

            %add zeros for channel delay
            tx = [tx_wave; zeros(trailing, cfgSU.NumTransmitAntennas)];
            
            %passing through the tgax channel 
            rx = TGAXchannel(tx);
            
            %apply path loss
            PL_linear = 10^(-PL_db / 20);
            rx = rx * PL_linear;

            %add noise
            rx = awgn(rx, snr(i));
            
            %estimate where the packet preamble begins
            startOffset = wlanPacketDetect(rx, bandwidth);

            %no valid packet or received packet is incomplete
            if isempty(startOffset) || startOffset + ind.HEData(2) > size(rx,1)
                error = error + 1;
                continue
            end
            
            %course cfo correction
            rxLSTF = rx(startOffset + (ind.LSTF(1) : ind.LSTF(2)),:);
            coarseCFO = wlanCoarseCFOEstimate(rxLSTF, bandwidth);
            rx = pfOffset(rx, -coarseCFO);

            %legacy parts: 'L-STF', 'L-LTF', 'L-SIG'
            legacy = rx(startOffset + (ind.LSTF(1) : ind.LSIG(2)),:);
            fineOffset = wlanSymbolTimingEstimate(legacy, bandwidth);

            pktOffset = startOffset + fineOffset;

            %if detect in the trailing zone, error and continue on next
            %packet
            if pktOffset > trailing
                error = error  + 1;
                continue
            end

            %fine cfo correction
            rxLLTF = rx(pktOffset + (ind.LLTF(1) : ind.LLTF(2)),:);
            fineCFO = wlanFineCFOEstimate(rxLLTF, bandwidth);
            rx = pfOffset(rx, -fineCFO);

            %isolate the HELTF and demodulate
            rxHELTF = rx(pktOffset + (ind.HELTF(1) : ind.HELTF(2)),:);
            DeHELTF = wlanHEDemodulate(rxHELTF, 'HE-LTF', cfgSU);

            %channel estimate at the high efficiency long training field
            [chanEst, EstPilots] = wlanHELTFChannelEstimate(DeHELTF, cfgSU);

            
            %isolate the HEData and demodulate
            rxData = rx(pktOffset + (ind.HEData(1) : ind.HEData(2)),:);
            DeRX = wlanHEDemodulate(rxData, 'HE-Data', cfgSU);
            
            %estimate the variance of additive white Gaussian noise for cfgSU
            X = DeRX(IPilot,:,:)
            noiseEst = wlanHEDataNoiseEstimate(X, EstPilots, cfgSU);
            
            %equalize the demodulated HE field symbols
            demodDataSym = DeRX(IData,:,:);
            chanEstData = chanEst(IData,:,:);
            [eqSym, csi] = wlanHEEqualize(demodDataSym, chanEstData, noiseEst, cfgSU, 'HE-Data');
            
            %recover bits
            PSDU_RX = wlanHEDataBitRecover(eqSym, noiseEst, csi, cfgSU, 'LDPCDecodingMethod', 'norm-min-sum');
            
            if any(biterr(PSDU_TX, PSDU_RX))
                error = error + 1;
            end
        end
        throughput(i) = apepBits * (1 - (error / (ipkt))) / txTime / 1e6;


    end
end