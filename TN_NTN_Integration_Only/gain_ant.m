function G = gain_ant(theta, theta_BS, theta_3dB_BS, SLA, G_UE)
G_E_max = 15; %[dBi]
G_E_dB = G_E_max - min(12 * ((theta - pi / 2) / theta_3dB_BS) .^ 2, SLA);
G_E = db2pow(G_E_dB);
M = 10; % With Mg = Ng = N = P = 1 and dV = lambda / 2, we simply end up with a single polarized uniform linear array (ULA) with the Array Factor (AF) equation as below.
AF = sin(M * pi / 2 * (cos(theta) - cos(theta_BS))) ./ (M * sin(pi / 2 * (cos(theta) - cos(theta_BS))));
AF(isnan(AF)) = 1;
G = G_UE .* G_E .* AF .^ 2;
end