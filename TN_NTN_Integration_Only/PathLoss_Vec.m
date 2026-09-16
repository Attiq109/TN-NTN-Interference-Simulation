function [PL_LoS, PL_NLoS] = PathLoss_Vec(Link, Mode, d_2D, d_3D, h_BS, h_UAV, h_UE, fc)
switch Link
    case {'BS2UE', 'UE2BS'}
        switch Mode %%% h_UE must be greater than 1.5 m
            case 'RMa' %%% BS antenna height should be ~35 m
                d_BP = 20 * pi * h_BS * h_UE * fc / 3;
                h = 5;
                W = 20;
                PL_LoS_dB = 20 * log10(40 * pi * d_3D * fc / 3) + min(10, 0.03 * h ^ 1.72) * log10(d_3D) - min(14.77, 0.044 * h ^ 1.72) + 0.002 * log10(h) * d_3D + 40 * log10(d_3D ./ d_BP) .* (d_2D > d_BP);
                PL_NLoS_dB = 161.04 - 7.1 * log10(W) + 7.5 * log10(h) - (24.37 - 3.7 * (h / h_BS) ^ 2) * log10(h_BS) + (43.42 - 3.1 * log10(h_BS)) * (log10(d_3D) - 3) + 20 * log10(fc) - (3.2 * log10(11.75 * h_UE) ^ 2 - 4.97);
                PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
            case 'UMa' %%% BS antenna height should be ~25 m
                d_BP = 40 * (h_BS - 1) * max(h_UE - 1, 0) * fc / 3; % approximated
                PL_LoS_dB = 28 + 22 * log10(d_3D) + 20 * log10(fc) + (18 * log10(d_3D) - 9 * log10(d_BP .^ 2 + (h_BS - h_UE) .^ 2)) .* (d_2D > d_BP);
                PL_NLoS_dB = 13.54 + 39.08 * log10(d_3D) + 20 * log10(fc) - 0.6 * max(h_UE - 1.5, 0);
                PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
            case 'UMi' %%% BS antenna height should be ~10 m
                d_BP = 40 * (h_BS - 1) * max(h_UE - 1, 0) * fc / 3;
                PL_LoS_dB = 32.4 + 21 * log10(d_3D) + 20 * log10(fc) + (19 * log10(d_3D) - 9.5 * log10(d_BP .^ 2 + (h_BS - h_UE) .^ 2)) .* (d_2D > d_BP);
                PL_NLoS_dB = 22.4 + 35.3 * log10(d_3D) + 21.3 * log10(fc) - 0.3 * max(h_UE - 1.5, 0);
                PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
    case {'BS2UAV', 'UAV2BS'}
        switch Mode
            case 'RMa' %%% BS antenna height should be ~35 m
                if mean(h_UAV) <= 10
                    [PL_LoS_dB, PL_NLoS_dB] = PathLoss_Vec('BS2UE', 'RMa', d_2D, d_3D, h_BS, [], h_UAV, fc);
                else
                    PL_LoS_dB = max(20, 23.9 - 1.8 * log10(h_UAV)) .* log10(d_3D) + 20 * log10(40 * pi * fc / 3);
                    PL_NLoS_dB = -12 + (35 - 5.3 * log10(h_UAV)) .* log10(d_3D) + 20 * log10(40 * pi * fc / 3);
                    PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
                end
            case 'UMa' %%% BS antenna height should be ~25 m
                if mean(h_UAV) <= 22.5
                    [PL_LoS_dB, PL_NLoS_dB] = PathLoss_Vec('BS2UE', 'UMa', d_2D, d_3D, h_BS, [], h_UAV, fc);
                else
                    PL_LoS_dB = 28 + 22 * log10(d_3D) + 20 * log10(fc);
                    PL_NLoS_dB = -17.5 + (46 - 7 * log10(h_UAV)) .* log10(d_3D) + 20 * log10(40 * pi * fc / 3);
                    PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
                end
            case 'UMi' %%% BS antenna height should be ~10 m
                if mean(h_UAV) <= 22.5
                    [PL_LoS_dB, PL_NLoS_dB] = PathLoss_Vec('BS2UE', 'UMi', d_2D, d_3D, h_BS, [], h_UAV, fc);
                else
                    FSPL = 20 * log10(40 * pi * d_3D * fc / 3);
                    PL_LoS_dB = 30.9 + (22.25 - 0.5 * log10(h_UAV)) .* log10(d_3D) + 20 * log10(fc);
                    PL_LoS_dB = max(FSPL, PL_LoS_dB);
                    PL_NLoS_dB = 32.4 + (43.2 - 7.6 * log10(h_UAV)) .* log10(d_3D) + 20 * log10(fc);
                    PL_NLoS_dB = max(PL_LoS_dB, PL_NLoS_dB);
                end
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
    case {'UAV2UE', 'UE2UAV'}
        switch Mode
            case 'RMa'
                eta_LoS = 0.1;
                eta_NLoS = 15;
            case 'UMa'
                eta_LoS = 1;
                eta_NLoS = 20;
            case 'UMi'
                eta_LoS = 2;
                eta_NLoS = 25;
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
        FSPL = 20 * log10(40 * pi * d_3D * fc / 3);
        PL_LoS_dB = FSPL + eta_LoS;
        PL_NLoS_dB = FSPL + eta_NLoS;
    case 'UAV2UAV'
        FSPL = 20 * log10(40 * pi * d_3D * fc / 3);
        PL_LoS_dB = FSPL;
        PL_NLoS_dB = FSPL;
    otherwise
        error('Invalid Link Type; Choose among BS2UE, BS2UAV, UAV2UE, or UAV2UAV.')
end
PL_LoS = db2pow(PL_LoS_dB);
PL_NLoS = db2pow(PL_NLoS_dB);