function p_att = rain_attenuation(d, R_point_01, gamma_R, alpha, fc) % From Recommendation ITU-R P.530-18
if fc == 6
    d = d / 1000; %[km]
    r = 1 ./ (0.477 * d .^ 0.633 * R_point_01 ^ (0.073 * alpha) * fc ^ 0.123 - 10.579 * (1 - exp(-0.024 * d)));
    d_eff = d .* r;
    p_att_dB = gamma_R * d_eff; % An estimate of the path attenuation exceeded for 0.01% of the time
    p_att = db2pow(p_att_dB);
else
    p_att = 1;
    warning('The parameters only work for fc = 6 GHz. For other frequencies, please update k and alpha from Table 5 of ITU-R P.838-3.')
end