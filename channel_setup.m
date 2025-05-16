function channel = channel_setup(ChannelType, band, samplerate, profile, numTX, numRX, speed)
    models = ["Model-A", "Model-B", "Model-C", "Model-D", "Model-E", "Model-F"];
    distance = [5 5 5 10 20 30]; %use the breakpoint distance for nlos
    d = dictionary(models, distance);
       
    if ChannelType == "TGAX"
        channel = wlanTGaxChannel;
    elseif ChannelType == "TGAC" 
        channel = wlanTGacChannel;
    end
    
    
    channel.SampleRate = samplerate;
    channel.ChannelBandwidth = band;
    channel.DelayProfile = profile;
    channel.TransmitReceiveDistance = d(profile);
    channel.NormalizeChannelOutputs = false;
    channel.NumTransmitAntennas = numTX;
    channel.NumReceiveAntennas = numRX;
    channel.EnvironmentalSpeed = speed;
end