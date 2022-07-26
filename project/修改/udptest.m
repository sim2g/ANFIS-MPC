udpr = dsp.UDPReceiver('LocalIPPort',2241);
setup(udpr);
dataReceived = udpr();
bytesReceived = length(dataReceived);