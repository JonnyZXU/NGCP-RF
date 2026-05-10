% Iteration 61
% Minimal 2-file setup:
%   1) this control script
%   2) iter61_engine.m
%
% Normal use:
%   - set RUN_BAYESOPT = false
%   - run this script
%
% Optional tuning:
%   - set RUN_BAYESOPT = true
%   - run this script

clear; clc; close all;

%% ===========================
% User switches
%% ===========================
RUN_BAYESOPT = false;
SHOW_PLOTS   = true;
SAVE_CSV     = true;
SAVE_CONFIDENCE_CSV = true;

NTRIALS = 5000;
rngSeed = 1;

%% ===========================
% Mission constants
%% ===========================
REGION_FT = 1500;
MAXVEC    = 20;
THRESH_FT = [20 30 40 50 100];

ORIGIN_LAT = 34.000000;
ORIGIN_LON = -118.000000;

%% ===========================
% Config
%% ===========================
cfg = make_iter61_cfg();
cfg.maxVec = MAXVEC;

if RUN_BAYESOPT
    vars = [
        optimizableVariable('planMovePenalty',[0.005 0.03],'Transform','log')
        optimizableVariable('planFarStepW',[0.05 0.60])
        optimizableVariable('firstProbe_ft',[10 35],'Type','integer')
        optimizableVariable('firstProbeRangeGate_ft',[20 70],'Type','integer')
        optimizableVariable('cameraNegW_mid',[0.10 0.95])
        optimizableVariable('pfResampleESSFrac',[0.25 0.90])
    ];

    results = bayesopt(@(x) iter61_objective(x), vars, ...
        'MaxObjectiveEvaluations', 24, ...
        'NumSeedPoints', 8, ...
        'IsObjectiveDeterministic', true, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'Verbose', 1, ...
        'PlotFcn', {@plotObjectiveModel, @plotMinObjective}); %#ok<NASGU>

    return;
end

%% ===========================
% Run simulation
%% ===========================
res = iter61_engine(cfg, NTRIALS, rngSeed);

%% ===========================
% Save CSV (feet + GPS)
%% ===========================
if SAVE_CSV
    [latPred, lonPred] = enu_ft_to_gps(res.pred_x_ft, res.pred_y_ft, ORIGIN_LAT, ORIGIN_LON);

    Tpred = table((1:NTRIALS)', ...
        res.start_x_ft, res.start_y_ft, ...
        res.tgt_x_ft, res.tgt_y_ft, res.spawn_target_dist_ft, ...
        res.pred_x_ft, res.pred_y_ft, res.pred_start_dist_ft, ...
        latPred, lonPred, ...
        res.final_err_ft, res.vec_used, ...
        res.conf_r95_ft, res.stop_trigger_conf_r95_ft, ...
        res.stop_trigger_pred_x_ft, res.stop_trigger_pred_y_ft, res.stop_trigger_err_ft, ...
        res.would_have_been_success_if_stopped_now, ...
        res.would_have_been_within30_if_stopped_now, ...
        res.would_have_been_within40_if_stopped_now, ...
        res.would_have_been_within50_if_stopped_now, ...
        res.would_have_been_within100_if_stopped_now, ...
        res.stop_by_conf, string(res.stop_reason), ...
        res.move_total_ft, string(res.move_steps_ft), ...
        res.scan_attempts_total, string(res.scan_attempts_steps), ...
        res.time_used_s, res.scan_time_s, res.travel_time_s, ...
        res.success20, res.success30, res.success40, res.success50, res.success100, ...
        'VariableNames', {'trial', ...
        'start_x_ft','start_y_ft', ...
        'tgt_x_ft','tgt_y_ft','spawn_target_dist_ft', ...
        'pred_x_ft','pred_y_ft','pred_start_dist_ft', ...
        'pred_lat','pred_lon', ...
        'final_err_ft','vec_used', ...
        'conf_r95_ft','stop_trigger_conf_r95_ft', ...
        'stop_trigger_pred_x_ft','stop_trigger_pred_y_ft','stop_trigger_err_ft', ...
        'would_have_been_success_if_stopped_now', ...
        'would_have_been_within30_if_stopped_now', ...
        'would_have_been_within40_if_stopped_now', ...
        'would_have_been_within50_if_stopped_now', ...
        'would_have_been_within100_if_stopped_now', ...
        'stop_by_conf','stop_reason', ...
        'move_total_ft','move_steps_ft', ...
        'scan_attempts_total','scan_attempts_steps', ...
        'time_used_s','scan_time_s','travel_time_s', ...
        'within20','within30','within40','within50','within100'});

    writetable(Tpred, 'mission_predictions.csv');
    fprintf("Saved final predictions to mission_predictions.csv\n\n");
end

%% ===========================
% Report
%% ===========================
report_results(res, MAXVEC);
[TconfSummary, TconfHits] = report_confidence_by_vector(res, MAXVEC, THRESH_FT);

if SAVE_CONFIDENCE_CSV
    writetable(TconfSummary, 'confidence_by_vector_summary.csv');
    writetable(TconfHits, 'confidence_hit_vectors.csv');
    fprintf("Saved confidence summaries to confidence_by_vector_summary.csv and confidence_hit_vectors.csv\n\n");
end

fprintf('\nExtra quick checks:\n');
falseConfRate = mean((res.conf_r95_ft <= cfg.stopConf_r95_ft) & (res.final_err_ft > 50), 'omitnan');
timeStopRate = mean(strcmp(res.stop_reason, 'time_before_move') | strcmp(res.stop_reason, 'time_before_scan'));
fprintf('  false-confidence proxy: %.2f%%\n', 100*falseConfRate);
fprintf('  time-stop rate:         %.2f%%\n', 100*timeStopRate);
timeLikeMask = strcmp(res.stop_reason, 'time_before_move') | strcmp(res.stop_reason, 'time_before_scan') | strcmp(res.stop_reason, 'time_before_camera_hit') | strcmp(res.stop_reason, 'time_before_camera_search') | strcmp(res.stop_reason, 'time_during_camera_search');
wouldNowMask = res.would_have_been_success_if_stopped_now;
if any(timeLikeMask)
    fprintf('  time-stop but already within20 now: %.2f%% of time-stops\n', 100*mean(wouldNowMask(timeLikeMask)));
end

%% ===========================
% Example plots
%% ===========================
if SHOW_PLOTS
    tid_best = pick_best_trial(res);
    fprintf("\nPlotting BEST within20 trial %d (err=%.1f ft)\n", tid_best, res.final_err_ft(tid_best));
    plot_trials_with_cones(res, tid_best, REGION_FT, cfg);

    tid_mid = pick_mid_trial(res);
    fprintf("Plotting within100-not50 trial %d (err=%.1f ft)\n", tid_mid, res.final_err_ft(tid_mid));
    plot_trials_with_cones(res, tid_mid, REGION_FT, cfg);

    tid_bad = pick_bad_trial(res);
    fprintf("Plotting fail>100 trial %d (err=%.1f ft)\n", tid_bad, res.final_err_ft(tid_bad));
    plot_trials_with_cones(res, tid_bad, REGION_FT, cfg);
end

fprintf('\nDone.\n');

%% =====================================================================
%% =========================== FUNCTIONS ===============================
%% =====================================================================

function cfg = make_iter61_cfg()
cfg = struct();

% Path loss (relative scale ok)
cfg.d0_m  = 1.0;
cfg.P0_dB = -35;
cfg.plExp = 2.2;

% Antenna / sweep shape
cfg.HPBW_deg = 70;

% Power variability
cfg.fadeSigma_dB = 2.5;
cfg.measSigma_dB = 2.0;
cfg.spikeProb    = 0.01;
cfg.spikeMag_dB  = 8.0;

% Servo bearing error
cfg.servoSigma_deg = 1.5;

% Cone sigma shaping
cfg.softmaxTau_dB   = 2.0;
cfg.sigmaFloor_deg  = 1.0;
cfg.sigmaCeil_deg   = 25.0;
cfg.minPeakProm_dB  = 3.0;
cfg.ambSigmaInflate = 2.5;

% Noise floor + saturation model
cfg.noiseFloor_dB   = -95;
cfg.noiseJitter_dB  = 1.0;
cfg.sigSat_dB       = -30;
cfg.satDistK        = 8.0;
cfg.satSigmaInflate = 1.5;
cfg.snrMin_dB       = -5;
cfg.snrMax_dB       = 60;

% Mission prior (annulus around start)
cfg.rMin_ft = 20;
cfg.rMax_ft = 500;

% Allow rover to roam a bit outside the annulus to get geometry
cfg.searchExtra_ft = 250;
cfg.searchRmax_ft  = cfg.rMax_ft + cfg.searchExtra_ft;

% Vehicle + servo constraints
cfg.servoLimit_deg  = 120;
cfg.servoMargin_deg = 10;
cfg.turnRadius_ft   = 40;

% Timing
cfg.timeBudget_s      = 1200;
cfg.travel_sec_per_ft = 60/120;
cfg.dwell_s           = 0.25;

% Scan model knobs
cfg.samplesPerAngle     = 8;
cfg.firstSweepStep_deg  = 3;
cfg.followSweepStep_deg = 1;

% Stage-2 fine sweep
cfg.fineHalfWidth_deg = 6;
cfg.fineStep_deg      = 0.5;

% Stage-2 disabling thresholds
cfg.fineDisableSig_deg   = 8;
cfg.fineMinSnr_dB        = 15;
cfg.fineDisableProm_dB   = 4;
cfg.stage2Policy         = "adaptiveSigma2";
cfg.fineVecMaxHard       = 100;

% Particle filter
cfg.pfN               = 5000;
cfg.pfResampleESSFrac = 0.55;
cfg.pfJitter_ft       = 6;
cfg.pfLikeFloor       = 1e-8;
cfg.pfSigmaScale      = 0.8;
cfg.pfLikeEps         = 0.01;

% Servo bias PF state
cfg.biasSigma0_deg   = 2.0;
cfg.pfBiasJitter_deg = 0.15;

% PF power fusion knobs
cfg.pfPowSigma0_dB     = 5.0;
cfg.pfPowJitter_dB     = 0.25;
cfg.pfPowSigma_dB      = 2.5;
cfg.pfPowLikeEps       = 0.1;
cfg.pfPowLikeFloor     = 1e-10;
cfg.pfPowOffClamp_dB   = 12.0;
cfg.pfPowResidClamp_dB = 12.0;

% Planner
cfg.minBaseline_ft = 60;
cfg.maxStep_ft     = 500;

cfg.planAngles      = 48;
cfg.planRadiiMult   = [0.6 0.9 1.2];
cfg.planMaxOrbit_ft = 300;
cfg.planSpacingCap_ft  = 300;
cfg.planMovePenalty    = 0.014;
cfg.unobsPenalty       = 2.0;
cfg.startRadiusPenalty = 0.002;
cfg.ringPenaltyW       = 0.003;

cfg.crossMinDeg_tail     = 20;
cfg.geomFixCooldownSteps = 2;

cfg.planBearBins        = 48;
cfg.planEntropyW        = 2.0;
cfg.planSplitW          = 1.6;
cfg.planCircVarW        = 0.9;
cfg.minVisibleMass      = 0.35;
cfg.planFarRange_ft     = 320;
cfg.planMaxOrbit_far_ft = 360;
cfg.planFarStepW        = 0.25;

cfg.plan2EntropyW = 2.0;
cfg.plan2SplitW   = 1.2;
cfg.plan2CrossW   = 1.8;

cfg.planPerpEntropyW = 1.6;
cfg.planPerpSplitW   = 1.0;
cfg.planPerpCrossW   = 1.2;
cfg.planCrossW       = 1.2;

% Scan consistency gating
cfg.consistencyEnable             = true;
cfg.consistencyMinAccepted        = 2;
cfg.consistencyBearingMassMin     = 0.05;
cfg.consistencyBearingGateMin_deg = 10;
cfg.consistencyBearingGateSigMult = 2.0;
cfg.consistencyPowerQlo           = 0.05;
cfg.consistencyPowerQhi           = 0.95;
cfg.consistencyPowerMargin_dB     = 2.0;
cfg.consistencyPromWeak_dB        = 4.0;
cfg.consistencyMaxRetries         = 1;
cfg.consistencySigmaInflate       = 1.8;

% Stop by confidence
cfg.stopConf_minVec = 2;
cfg.stopConf_r95_ft = 24;
cfg.stopRequireCameraConsistency = true;
cfg.stopCameraVisibleMassMax     = 0.20;
cfg.confirmScansOnStop           = 3;

% Final localization mode
cfg.finalModeTrigger_r95_ft = 130;
cfg.finalModeMaxOrbit_ft    = 90;
cfg.finalModeRadiiMult      = [0.55 0.75 0.95];
cfg.finalModeMaxStep_ft     = 100;
cfg.finalCameraMassW        = 2.0;

% Extra scans usage
cfg.extraEndScansMax   = 8;
cfg.extraScanHalfW_deg = 25;
cfg.extraScanStep_deg  = 2;

% Camera model
cfg.cameraDetect_ft     = 20;
cfg.cameraDetectProb    = 0.95;
cfg.cameraMissProb      = 0.05;
cfg.cameraUseNegative   = true;
cfg.cameraNegativeW     = 0.15;
cfg.cameraFalseAlarm    = 0.00;
cfg.cameraSampleStep_ft = 2;

cfg.cameraNegW_far   = 0.20;
cfg.cameraNegW_mid   = 0.45;
cfg.cameraNegW_close = 0.80;

cfg.cameraNegMid_ft   = 60;
cfg.cameraNegClose_ft = 30;

% First probe after first accepted vector
cfg.firstProbeEnable       = true;
cfg.firstProbe_ft          = 25;
cfg.firstProbeRangeGate_ft = 35;

% Early geometry fix
cfg.crossMinDeg_early       = 35;
cfg.earlyGeomFixBaseline_ft = 160;

% Camera-search handoff mode
cfg.cameraSearchEnable             = true;
cfg.cameraSearchMinVec             = 4;
cfg.cameraSearchR95Trigger_ft      = 50;
cfg.cameraSearchConfidencePct      = 90;
cfg.cameraSearchEffWidth_ft        = 30;
cfg.cameraSearchCoverageEff        = 0.80;
cfg.cameraSearchTimeBuffer_s       = 45;
cfg.cameraSearchGuardRadius_ft     = 35;
cfg.cameraSearchGuardAngles_deg    = [-90 0 90];
cfg.cameraSearchGuardSpreadMax_dB  = 4.0;
cfg.cameraSearchGuardCenterSlack_dB = 2.0;
cfg.cameraSearchGuardMinSnr_dB     = 12.0;
cfg.cameraSearchMaxAttempts        = 1;
cfg.cameraSearchSpacing_ft         = 30;
cfg.cameraSearchRegionFloor_ft     = 20;

% Saturation-aware handoff
cfg.rfSaturationEnable            = true;
cfg.rfSaturationHandoff_ft        = 125;
cfg.rfSaturationMax_ft            = 150;
cfg.cameraSearchSatRegionTrigger_ft = 85;
cfg.cameraSearchSatMinVec         = 4;
cfg.cameraSearchSatTimeBuffer_s   = 60;

cfg.debug = false;
end

function objective = iter61_objective(x)
cfg = make_iter61_cfg();
cfg.maxVec = 20;

cfg.planMovePenalty      = x.planMovePenalty;
cfg.planFarStepW         = x.planFarStepW;
cfg.firstProbe_ft        = x.firstProbe_ft;
cfg.firstProbeRangeGate_ft = x.firstProbeRangeGate_ft;
cfg.cameraNegW_mid       = x.cameraNegW_mid;
cfg.pfResampleESSFrac    = x.pfResampleESSFrac;

seeds = [1 2 3];
allWithin20 = zeros(size(seeds));
allWithin50 = zeros(size(seeds));
allFail100  = zeros(size(seeds));
allFalseConf = zeros(size(seeds));
allTimeStop  = zeros(size(seeds));
allTime      = zeros(size(seeds));

for i = 1:numel(seeds)
    res = iter61_engine(cfg, 300, seeds(i));
    allWithin20(i) = mean(res.success20);
    allWithin50(i) = mean(res.success50);
    allFail100(i)  = mean(res.final_err_ft > 100);
    allFalseConf(i) = mean((res.conf_r95_ft <= cfg.stopConf_r95_ft) & (res.final_err_ft > 50), 'omitnan');
    allTimeStop(i) = mean(strcmp(res.stop_reason, 'time_before_move') | strcmp(res.stop_reason, 'time_before_scan'));
    allTime(i) = median(res.time_used_s);
end

within20  = mean(allWithin20);
within50  = mean(allWithin50);
fail100   = mean(allFail100);
falseConf = mean(allFalseConf);
timeStop  = mean(allTimeStop);
medianTime = mean(allTime);

objective = ...
    -4.0 * within20 ...
    -1.5 * within50 ...
    +2.5 * fail100 ...
    +1.5 * falseConf ...
    +1.5 * timeStop ...
    +0.0005 * medianTime;

fprintf(['Eval: w20=%.4f w50=%.4f fail100=%.4f falseConf=%.4f ', ...
         'timeStop=%.4f obj=%.4f\n'], ...
        within20, within50, fail100, falseConf, timeStop, objective);
end

function [lat, lon] = enu_ft_to_gps(x_ft, y_ft, lat0, lon0)
x_m = x_ft * 0.3048;
y_m = y_ft * 0.3048;
phi = deg2rad(lat0);
m_per_deg_lat = 111132.92 - 559.82*cos(2*phi) + 1.175*cos(4*phi) - 0.0023*cos(6*phi);
m_per_deg_lon = 111412.84*cos(phi) - 93.5*cos(3*phi) + 0.118*cos(5*phi);
lat = lat0 + (y_m ./ m_per_deg_lat);
lon = lon0 + (x_m ./ m_per_deg_lon);
end

function report_results(res, MAXVEC)
fprintf("Trials: %d\n", numel(res.success20));
fprintf("Within 20 ft: %d (%.1f%%)\n", sum(res.success20), 100*mean(res.success20));
fprintf("Within 30 ft: %d (%.1f%%)\n", sum(res.success30), 100*mean(res.success30));
fprintf("Within 40 ft: %d (%.1f%%)\n", sum(res.success40), 100*mean(res.success40));
fprintf("Within 50 ft:  %d (%.1f%%)\n", sum(res.success50), 100*mean(res.success50));
fprintf("Within 100 ft: %d (%.1f%%)\n", sum(res.success100), 100*mean(res.success100));

fprintf("Camera-found trials: %d (%.1f%%)\n", sum(res.camera_found), 100*mean(res.camera_found));

if isfield(res,'camera_search_triggered')
    fprintf("Camera-search triggered: %d (%.1f%%)\n", sum(res.camera_search_triggered), 100*mean(res.camera_search_triggered));
end
if isfield(res,'camera_search_found')
    fprintf("Camera-search found:     %d (%.1f%%)\n", sum(res.camera_search_found), 100*mean(res.camera_search_found));
end

fprintf("\nScan attempts:\n");
fprintf("  median attempts (all trials): %.1f\n", median(res.scan_attempts_total));
if any(res.success20)
    fprintf("  median attempts (within20):   %.1f\n", median(res.scan_attempts_total(res.success20)));
else
    fprintf("  median attempts (within20):   n/a\n");
end

denAll = max(res.vec_used, 1);
fprintf("  median attempts per accepted vector (all): %.2f\n", median(res.scan_attempts_total ./ denAll));
if any(res.success20)
    den20  = max(res.vec_used(res.success20), 1);
    fprintf("  median attempts per accepted vector (within20): %.2f\n", median(res.scan_attempts_total(res.success20) ./ den20));
end

succIdx = find(res.success20);
vecUsedSucc = res.vec_used(succIdx);
fprintf("\nBreakdown (success<=20 ft) by vectors used:\n");
for kk = 1:MAXVEC
    c = sum(vecUsedSucc == kk);
    if c > 0
        fprintf("  %2d vectors: %4d\n", kk, c);
    end
end

fprintf("\nMovement:\n");
fprintf("  median move_total_ft (all):     %.1f\n", median(res.move_total_ft));
if any(res.success20)
    fprintf("  median move_total_ft (within20):%.1f\n", median(res.move_total_ft(res.success20)));
else
    fprintf("  median move_total_ft (within20):n/a\n");
end

fprintf("\nTime:\n");
fprintf("  median time_used_s (all):      %.1f\n", median(res.time_used_s));
fprintf("  median scan_time_s (all):      %.1f\n", median(res.scan_time_s));
fprintf("  median travel_time_s (all):    %.1f\n", median(res.travel_time_s));
if any(res.success20)
    fprintf("  median time_used_s (within20): %.1f\n", median(res.time_used_s(res.success20)));
else
    fprintf("  median time_used_s (within20): n/a\n");
end
end

function [Tsummary, Thits] = report_confidence_by_vector(res, MAXVEC, THRESH_FT)
N = numel(res.vec_used);
M = numel(THRESH_FT);

firstHit = nan(N, M);

for t = 1:N
    if ~isfield(res, 'hist_r95') || numel(res.hist_r95) < t || isempty(res.hist_r95{t})
        continue;
    end

    r95hist = res.hist_r95{t};
    r95hist = r95hist(:);

    for j = 1:M
        kHit = find(r95hist <= THRESH_FT(j), 1, 'first');
        if ~isempty(kHit)
            firstHit(t, j) = kHit;
        end
    end
end

fprintf('\nConfidence radius by accepted vector (r95 threshold):\n');
summaryRows = zeros(M, 4 + MAXVEC);

for j = 1:M
    thr = THRESH_FT(j);
    hitMask = ~isnan(firstHit(:,j));

    reachedTrials = sum(hitMask);
    reachedPct = 100 * mean(hitMask);

    if any(hitMask)
        medVec = median(firstHit(hitMask,j));
        p25Vec = prctile(firstHit(hitMask,j), 25);
        p75Vec = prctile(firstHit(hitMask,j), 75);
    else
        medVec = NaN;
        p25Vec = NaN;
        p75Vec = NaN;
    end

    cumulativePct = zeros(1, MAXVEC);
    for k = 1:MAXVEC
        cumulativePct(k) = 100 * mean(firstHit(:,j) <= k);
    end

    fprintf('  r95 <= %3d ft: reached in %4d/%d trials (%.1f%%), median first vector = %.1f\n', ...
        thr, reachedTrials, N, reachedPct, medVec);
    fprintf('     cumulative by vector: ');
    for k = 1:MAXVEC
        if k < MAXVEC
            fprintf('v%d %.1f%%, ', k, cumulativePct(k));
        else
            fprintf('v%d %.1f%%\n', k, cumulativePct(k));
        end
    end

    summaryRows(j,:) = [thr, reachedTrials, reachedPct, medVec, cumulativePct];
end

summaryNames = [{'threshold_ft','reached_trials','reached_pct','median_first_vector'}, ...
                arrayfun(@(k) sprintf('by_vec_%d_pct', k), 1:MAXVEC, 'UniformOutput', false)];
Tsummary = array2table(summaryRows, 'VariableNames', summaryNames);

hitNames = arrayfun(@(thr) sprintf('first_vec_r95_le_%dft', thr), THRESH_FT, 'UniformOutput', false);
Thits = array2table([(1:N)', firstHit], 'VariableNames', [{'trial'}, hitNames]);
end


function plot_trials_with_cones(res, tid, REGION_FT, cfg)
figure('Color','w'); hold on; grid on; axis equal;

plot([0 REGION_FT REGION_FT 0 0],[0 0 REGION_FT REGION_FT 0],'k-','LineWidth',1.5,'DisplayName','Region');

pS = [res.start_x_ft(tid), res.start_y_ft(tid)];
pT = [res.tgt_x_ft(tid),   res.tgt_y_ft(tid)];
pP = [res.pred_x_ft(tid),  res.pred_y_ft(tid)];

th = linspace(0,2*pi,400);
r1 = cfg.rMin_ft;
r2 = cfg.rMax_ft;
plot(pS(1)+r1*cos(th), pS(2)+r1*sin(th),'--','LineWidth',1.5,'DisplayName','Start rMin');
plot(pS(1)+r2*cos(th), pS(2)+r2*sin(th),'--','LineWidth',1.5,'DisplayName','Start rMax');

plot(pS(1),pS(2),'o','MarkerSize',12,'LineWidth',2,'DisplayName','START');
plot(pT(1),pT(2),'*','MarkerSize',14,'LineWidth',2,'DisplayName','TARGET');
plot(pP(1),pP(2),'x','MarkerSize',12,'LineWidth',3,'DisplayName','PRED');

pos = res.hist_pos{tid};
thh = res.hist_th{tid};
sig = res.hist_sig{tid};

if isempty(pos)
    title(sprintf("Trial %d (no scans?)", tid));
    return;
end

plot(pos(:,1), pos(:,2), '-','LineWidth',2,'DisplayName','Travel');
for i = 1:size(pos,1)
    plot(pos(i,1),pos(i,2),'o','MarkerSize',10,'LineWidth',2,'DisplayName', ternary(i==1,"Scan locations",""));
    text(pos(i,1)+8, pos(i,2)+8, sprintf("%d", i), 'FontSize',11, 'FontWeight','bold');
end

for i = 1:(size(pos,1)-1)
    dp = pos(i+1,:) - pos(i,:);
    quiver(pos(i,1),pos(i,2),dp(1),dp(2),0,'LineWidth',1.5,'MaxHeadSize',0.7,'DisplayName',ternary(i==1,"Travel direction",""));
end

outerC = pS;
outerR = cfg.rMax_ft;

for i = 1:size(pos,1)
    if ~isfinite(thh(i)) || ~isfinite(sig(i)), continue; end

    t0 = ray_circle_hit(pos(i,:), thh(i), outerC, outerR);
    if isfinite(t0)
        v = t0 * [cosd(thh(i)) sind(thh(i))];
        plot([pos(i,1) pos(i,1)+v(1)], [pos(i,2) pos(i,2)+v(2)], '-', ...
            'LineWidth', 1.2, ...
            'DisplayName', ternary(i==1,"Cone centerline",""));
    end

    a1 = thh(i) - 2*sig(i);
    a2 = thh(i) + 2*sig(i);

    [xp, yp] = cone_patch_to_circle(pos(i,:), a1, a2, outerC, outerR, 80);
    if isempty(xp), continue; end

    patch(xp, yp, [0.2 0.6 1.0], ...
        'FaceAlpha', 0.12, ...
        'EdgeColor', [0.2 0.6 1.0], ...
        'LineWidth', 1.0, ...
        'DisplayName', ternary(i==1,"Scan cones",""));
end

xlim([max(0,min([pos(:,1);pS(1);pT(1);pP(1)])-120) , min(REGION_FT,max([pos(:,1);pS(1);pT(1);pP(1)])+120)]);
ylim([max(0,min([pos(:,2);pS(2);pT(2);pP(2)])-120) , min(REGION_FT,max([pos(:,2);pS(2);pT(2);pP(2)])+120)]);

err = res.final_err_ft(tid);
title(sprintf("Trial %d | err=%.1f ft | within20=%d | vec_used=%d | stop=%s", ...
    tid, err, res.success20(tid), res.vec_used(tid), string(res.stop_reason{tid})));

legend('Location','bestoutside');
xlabel('x (ft)'); ylabel('y (ft)');
end

function tid = pick_best_trial(res)
idx = find(res.success20);
if isempty(idx)
    [~, tid] = min(res.final_err_ft);
    return;
end
[~, j] = min(res.final_err_ft(idx));
tid = idx(j);
end

function tid = pick_mid_trial(res)
err = res.final_err_ft;
idx = find(err > 50 & err <= 100);
if isempty(idx)
    idx = find(err <= 100);
end
[~, j] = min(abs(err(idx) - 75));
tid = idx(j);
end

function tid = pick_bad_trial(res)
err = res.final_err_ft;
idx = find(err > 100);
if isempty(idx)
    [~, tid] = max(err);
    return;
end
[~, j] = min(abs(err(idx) - 150));
tid = idx(j);
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function tHit = ray_circle_hit(p0, ang_deg, c, r)
d = [cosd(ang_deg), sind(ang_deg)];
f = p0 - c;

b = 2 * dot(d, f);
cc = dot(f, f) - r^2;

disc = b^2 - 4*cc;
if disc < 0
    tHit = NaN;
    return;
end

sdisc = sqrt(disc);
t1 = (-b - sdisc) / 2;
t2 = (-b + sdisc) / 2;

ts = sort([t1 t2]);
ts = ts(ts > 1e-6);

if isempty(ts)
    tHit = NaN;
else
    tHit = ts(1);
end
end

function [xp, yp] = cone_patch_to_circle(p0, a1_deg, a2_deg, c, r, nPts)
angs = linspace(a1_deg, a2_deg, nPts);
xArc = [];
yArc = [];

for k = 1:numel(angs)
    tHit = ray_circle_hit(p0, angs(k), c, r);
    if isfinite(tHit)
        pt = p0 + tHit * [cosd(angs(k)), sind(angs(k))];
        xArc(end+1) = pt(1); %#ok<AGROW>
        yArc(end+1) = pt(2); %#ok<AGROW>
    end
end

if numel(xArc) < 2
    xp = [];
    yp = [];
    return;
end

xp = [p0(1), xArc, p0(1)];
yp = [p0(2), yArc, p0(2)];
end
