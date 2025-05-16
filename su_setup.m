function cfgSu = su_setup(type, bandwidth, spatialmapping, numTX, numST, APEPlength, ...
    GuardInterval, MCS)
    
    if type == "HE"
        cfgSu = wlanHESUConfig;
    elseif type == "VHT"
        cfgSu = wlanVHTConfig;
    end
    
    cfgSu.ChannelCoding = 'LDPC';
    cfgSu.ChannelBandwidth = bandwidth;
    cfgSu.SpatialMapping = spatialmapping;
    cfgSu.NumTransmitAntennas = numTX;
    cfgSu.NumSpaceTimeStreams = numST;
    cfgSu.APEPLength = APEPlength;
    cfgSu.GuardInterval = GuardInterval;
    cfgSu.MCS = MCS;
end