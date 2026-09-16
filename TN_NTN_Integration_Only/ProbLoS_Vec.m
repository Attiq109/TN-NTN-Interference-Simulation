function PrLoS = ProbLoS_Vec(Link, Mode, d_2D, d_3D, h_BS, h_UAV, h_UE)
switch Link
    case {'BS2UE', 'UE2BS'}
        switch Mode %%% h_UE must be greater than 1.5 m
            case 'RMa' %%% BS antenna height should be ~35 m
                PrLoS = exp(-(d_2D - 10) / 1000) .* (d_2D > 10) + 1 * (d_2D <= 10);
            case 'UMa' %%% BS antenna height should be ~25 m
                C = ((h_UE - 13) / 10) .^ 1.5 .* (h_UE > 13);
                PrLoS = (18 ./ d_2D + (1 - 18 ./ d_2D) .* exp(-d_2D / 63)) .* (1 + C .* 5 ./ 4 .* (d_2D / 100) .^ 3 .* exp(-d_2D / 150)) .* (d_2D > 18) + 1 * (d_2D <= 18);
            case 'UMi' %%% BS antenna height should be ~10 m
                PrLoS = (18 ./ d_2D + (1 - 18 ./ d_2D) .* exp(-d_2D / 36)) .* (d_2D > 18) + 1 * (d_2D <= 18);
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
    case {'BS2UAV', 'UAV2BS'}
        switch Mode
            case 'RMa' %%% BS antenna height should be ~35 m
                if mean(h_UAV) <= 10
                    PrLoS = ProbLoS_Vec('BS2UE', 'RMa', d_2D, d_3D, h_BS, [], h_UAV);
                elseif (mean(h_UAV) > 10) && (mean(h_UAV) <= 40)
                    p1 = max(1000, 15021 * log10(h_UAV) - 16053);
                    d1 = max(18, 1350.8 * log10(h_UAV) - 1602);
                    PrLoS = (d1 ./ d_2D + (1 - d1 ./ d_2D) .* exp(-d_2D ./ p1)) .* (d_2D > d1) + 1 * (d_2D <= d1);
                else
                    PrLoS = 1;
                end
            case 'UMa' %%% BS antenna height should be ~25 m
                if mean(h_UAV) <= 22.5
                    PrLoS = ProbLoS_Vec('BS2UE', 'UMa', d_2D, d_3D, h_BS, [], h_UAV);
                elseif (mean(h_UAV) > 22.5) && (mean(h_UAV) <= 100)
                    p1 = 4300 * log10(h_UAV) - 3800;
                    d1 = max(18, 460 * log10(h_UAV) - 700);
                    PrLoS = (d1 ./ d_2D + (1 - d1 ./ d_2D) .* exp(-d_2D ./ p1)) .* (d_2D > d1) + 1 * (d_2D <= d1);
                else
                    PrLoS = 1;
                end
            case 'UMi' %%% BS antenna height should be ~10 m
                if mean(h_UAV) <= 22.5
                    PrLoS = ProbLoS_Vec('BS2UE', 'UMi', d_2D, d_3D, h_BS, [], h_UAV);
                else
                    p1 = 233.98 * log10(h_UAV) - 0.95;
                    d1 = max(18, 294.05 * log10(h_UAV) - 432.94);
                    PrLoS = (d1 ./ d_2D + (1 - d1 ./ d_2D) .* exp(-d_2D ./ p1)) .* (d_2D > d1) + 1 * (d_2D <= d1);
                end
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
    case {'UAV2UE', 'UE2UAV'}
        switch Mode
            case 'RMa'
                alpha = 0.05;
                beta = 200;
                gamma = 8;
            case 'UMa'
                alpha = 0.3;
                beta = 500;
                gamma = 15;
            case 'UMi'
                alpha = 0.3;
                beta = 500;
                gamma = 15;
            otherwise
                error('Invalid Mode; Choose among RMa, UMa, or UMi.')
        end
        aC = [9.34e-1, 1.97e-2, -1.24e-4, 2.73e-7; 2.30e-1, 2.44e-3, -3.34e-6, 0; -2.25e-3, 6.58e-6, 0, 0; 1.86e-5, 0, 0, 0].';
        bC = [1.17e0, -5.79e-3, 1.73e-5, -2.00e-8; -7.56e-2, 1.81e-4, -2.02e-7, 0; 1.98e-3, -1.65e-6, 0, 0; -1.78e-5, 0, 0, 0].';
        a = 0;
        b = 0;
        for j = 0 : 3
            for i = 0 : 3 - j
                a = a + aC(i + 1, j + 1) * (alpha * beta) ^ i * gamma ^ j;
                b = b + bC(i + 1, j + 1) * (alpha * beta) ^ i * gamma ^ j;
            end
        end
        theta = asind((h_UAV - h_UE) ./ d_3D);
        PrLoS = 1 ./ (1 + a * exp(-b * (theta - a)));
    case 'UAV2UAV'
        PrLoS = 1;
    otherwise
        error('Invalid Link Type; Choose among BS2UE, BS2UAV, UAV2UE, or UAV2UAV.')
end