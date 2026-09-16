function G = gain_ant_NTN(theta, theta_3dB_UE_NTN, SLA, G_UE)
G_E_max = 15; %[dBi]
theta_boresight = pi;
G_E_dB = G_E_max - min(12 * ((theta - theta_boresight) / theta_3dB_UE_NTN) .^ 2, SLA);
G_E = db2pow(G_E_dB);
G = G_UE .* G_E;
end
