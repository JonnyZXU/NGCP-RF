function res = iter61_engine(cfg, NTRIALS, rngSeed)
% ITER61_ENGINE
% Saturation-aware simulation engine used by Iteration_61.

if nargin < 1 || isempty(cfg)
    error('iter61_engine requires cfg. Build cfg with make_iter61_cfg() inside Iteration_61.m.');
end
if nargin < 2 || isempty(NTRIALS)
    NTRIALS = 1000;
end
if nargin < 3 || isempty(rngSeed)
    rngSeed = 1;
end

rng(rngSeed);

REGION_FT = 1500;
if nargin < 1 || isempty(cfg)
    cfg = struct();
end
if isfield(cfg,'maxVec') && ~isempty(cfg.maxVec)
    MAXVEC = cfg.maxVec;
else
    MAXVEC = 10;
end
THRESH_FT = [20 30 40 50 100];
START_MODE = "random";
START_GRID_STEP_FT = 10;

res = run_trials(REGION_FT, NTRIALS, MAXVEC, THRESH_FT, START_MODE, START_GRID_STEP_FT, cfg);
end

function res = run_trials(REGION_FT, NTRIALS, MAXVEC, THRESH_FT, START_MODE, START_GRID_STEP_FT, cfg)

res.start_x_ft = zeros(NTRIALS,1);
res.start_y_ft = zeros(NTRIALS,1);
res.tgt_x_ft   = zeros(NTRIALS,1);
res.tgt_y_ft   = zeros(NTRIALS,1);

res.pred_x_ft    = nan(NTRIALS,1);
res.pred_y_ft    = nan(NTRIALS,1);
res.final_err_ft = nan(NTRIALS,1);
res.vec_used     = zeros(NTRIALS,1);

res.success20  = false(NTRIALS,1);
res.success30  = false(NTRIALS,1);
res.success40  = false(NTRIALS,1);
res.success50  = false(NTRIALS,1);
res.success100 = false(NTRIALS,1);

res.move_total_ft = zeros(NTRIALS,1);
res.move_steps_ft = cell(NTRIALS,1);

res.scan_attempts_total = zeros(NTRIALS,1);
res.scan_attempts_steps = cell(NTRIALS,1);

res.conf_r95_ft  = nan(NTRIALS,1);
res.stop_by_conf = false(NTRIALS,1);
res.stop_reason  = cell(NTRIALS,1);
res.stop_trigger_conf_r95_ft = nan(NTRIALS,1);
res.stop_trigger_pred_x_ft   = nan(NTRIALS,1);
res.stop_trigger_pred_y_ft   = nan(NTRIALS,1);
res.stop_trigger_err_ft      = nan(NTRIALS,1);
res.would_have_been_success_if_stopped_now   = false(NTRIALS,1);
res.would_have_been_within30_if_stopped_now  = false(NTRIALS,1);
res.would_have_been_within40_if_stopped_now  = false(NTRIALS,1);
res.would_have_been_within50_if_stopped_now  = false(NTRIALS,1);
res.would_have_been_within100_if_stopped_now = false(NTRIALS,1);

res.time_used_s   = zeros(NTRIALS,1);
res.scan_time_s   = zeros(NTRIALS,1);
res.travel_time_s = zeros(NTRIALS,1);

% store per-trial history for plotting
res.hist_pos   = cell(NTRIALS,1);
res.hist_psi   = cell(NTRIALS,1);
res.hist_th    = cell(NTRIALS,1);
res.hist_sig   = cell(NTRIALS,1);
res.hist_snr   = cell(NTRIALS,1);
res.hist_prom  = cell(NTRIALS,1);
res.hist_halfW = cell(NTRIALS,1);
res.hist_step  = cell(NTRIALS,1);
res.hist_r95   = cell(NTRIALS,1);

res.spawn_target_dist_ft = nan(NTRIALS,1);
res.pred_start_dist_ft   = nan(NTRIALS,1);

res.camera_found = false(NTRIALS,1);
res.camera_search_triggered = false(NTRIALS,1);
res.camera_search_found     = false(NTRIALS,1);
res.travel_seg_start = cell(NTRIALS,1);
res.travel_seg_end   = cell(NTRIALS,1);
res.travel_seg_type  = cell(NTRIALS,1);
res.travel_seg_hit   = cell(NTRIALS,1);

% Start positions
if START_MODE == "grid"
    xs = 0:START_GRID_STEP_FT:REGION_FT;
else
    xs = [];
end

for t = 1:NTRIALS
    timeUsed_s = 0;
    scanTime_s = 0;
    travelTime_s = 0;

    cameraFound  = false;
    cameraHitPos = [NaN NaN];
    cameraSearchTriggered = false;
    cameraSearchFound = false;
    cameraSearchAttempts = 0;

    segStart = zeros(0,2);
    segEnd   = zeros(0,2);
    segType  = strings(0,1);
    segHit   = false(0,1);

    % start (bottom edge)
    if START_MODE == "grid"
        x0 = xs(1 + mod(t-1, numel(xs)));
    else
        x0 = REGION_FT * rand;
    end
    pStart = [x0, 0];
    p = pStart;
    psi = 90;

    res.start_x_ft(t) = pStart(1);
    res.start_y_ft(t) = pStart(2);

    % target in annulus around start
    [tx, ok] = sample_target_annulus_in_region(pStart, REGION_FT, cfg.rMin_ft, cfg.rMax_ft);
    while ~ok
        [tx, ok] = sample_target_annulus_in_region(pStart, REGION_FT, cfg.rMin_ft, cfg.rMax_ft);
    end
    res.tgt_x_ft(t) = tx(1);
    res.tgt_y_ft(t) = tx(2);
    res.spawn_target_dist_ft(t) = hypot(tx(1)-pStart(1), tx(2)-pStart(2));

    % history buffers
    measPos   = zeros(MAXVEC,2);
    measPsi   = nan(MAXVEC,1);
    measTh    = nan(MAXVEC,1);
    measSig   = nan(MAXVEC,1);
    measSnr   = nan(MAXVEC,1);
    measProm  = nan(MAXVEC,1);
    measHalfW = nan(MAXVEC,1);
    measStep  = nan(MAXVEC,1);
    measRange = nan(MAXVEC,1);
    confR95Hist = nan(MAXVEC,1);

    stepDists = nan(MAXVEC-1,1);
    stepCount = 0;

    scanAttemptsTotal = 0;
    attemptsPerVec = nan(MAXVEC,1);
    attemptsThisVec = 0;
    acceptedCount = 0;

    % PF init: particles are [x,y,bias_deg,pOff_dB]
    [pf_xyb, pf_w] = pf_init_particles_annulus_bias( ...
        pStart, REGION_FT, cfg.rMin_ft, cfg.rMax_ft, cfg.pfN, ...
        cfg.biasSigma0_deg, cfg.pfPowSigma0_dB);

    pred_pf = [nan nan];
    r95 = inf;
    stopReason = "maxvec";
    retry = 0;
    geomCooldown = 0;

    k = 1;
    while k <= MAXVEC

        [halfW, stepD, ~] = select_sweep_profile(cfg, k, retry, acceptedCount, measSig, measSnr, measProm);

        if acceptedCount >= 1 && isfinite(measTh(acceptedCount))
            centerGuess_rel = wrap180(measTh(acceptedCount) - psi);
        else
            centerGuess_rel = 0;
        end

        cfgScan = make_scan_cfg(cfg, k, acceptedCount, measSig, measSnr, measProm);

        scanT = scan_time_seconds(halfW, stepD, cfgScan);
        if timeUsed_s + scanT > cfg.timeBudget_s
            stopReason = "time_before_scan";
            break;
        end

        timeUsed_s = timeUsed_s + scanT;
        scanTime_s = scanTime_s + scanT;

        scanAttemptsTotal = scanAttemptsTotal + 1;
        attemptsThisVec   = attemptsThisVec + 1;

        [th_hat_global, sig_hat, pmax_dB, snrMax_dB, prom_dB] = scan_bearing_from_power_heading( ...
            p, psi, tx, centerGuess_rel, halfW, stepD, cfgScan);

        if cfg.debug
            doFine = (cfgScan.fineHalfWidth_deg > 0) && (cfgScan.fineStep_deg > 0);
            fprintf("t=%d k=%d doFine=%d halfW=%.1f step=%.1f sig=%.2f snr=%.1f prom=%.1f\n", ...
                t, k, doFine, halfW, stepD, sig_hat, snrMax_dB, prom_dB);
        end

        % reject/retry thresholds
        sigRejectK  = (k <= 2) * 20 + (k > 2) * 18;
        maxRetriesK = 1;

        % Use this for PF update; keep original pmax_dB for range planning
        pmaxForPF_dB = pmax_dB;
        skipSigmaReject = false;

        % PF consistency gating
        if isfield(cfg,'consistencyEnable') && cfg.consistencyEnable && ...
           (acceptedCount >= cfg.consistencyMinAccepted)

            Pnow = pf_xyb(:,1:2);
            wnow = pf_w(:);
            sw = sum(wnow);

            if isfinite(sw) && sw > 0
                wnow = wnow / sw;

                % Bearing consistency
                th_pred = wrap180(atan2d(Pnow(:,2)-p(2), Pnow(:,1)-p(1)) + pf_xyb(:,3));

                gateDeg = max(cfg.consistencyBearingGateMin_deg, ...
                              cfg.consistencyBearingGateSigMult * sig_hat);

                dth = abs(wrap180(th_pred - th_hat_global));
                massNear = sum(wnow(dth <= gateDeg));
                bearingBad = massNear < cfg.consistencyBearingMassMin;

                % Power consistency
                d_ft_pf = hypot(Pnow(:,1)-p(1), Pnow(:,2)-p(2));
                d_m_pf  = max(d_ft_pf*0.3048, 0.05);
                p_pred_pf = (cfg.P0_dB + pf_xyb(:,4)) - 10*cfg.plExp*log10(d_m_pf / cfg.d0_m);

                pLo = weighted_quantile(p_pred_pf, wnow, cfg.consistencyPowerQlo);
                pHi = weighted_quantile(p_pred_pf, wnow, cfg.consistencyPowerQhi);

                powerBad = isfinite(pmax_dB) && ...
                          ((pmax_dB < pLo - cfg.consistencyPowerMargin_dB) || ...
                           (pmax_dB > pHi + cfg.consistencyPowerMargin_dB));

                % Peak quality
                promWeak = ~isfinite(prom_dB) || (prom_dB < cfg.consistencyPromWeak_dB);

                inconsistent = (bearingBad && powerBad) || (bearingBad && promWeak);

                if inconsistent
                    retry = retry + 1;

                    if retry <= cfg.consistencyMaxRetries
                        continue;
                    else
                        sig_hat = min(cfg.sigmaCeil_deg, cfg.consistencySigmaInflate * sig_hat);
                        pmaxForPF_dB = NaN;
                        skipSigmaReject = true;
                        retry = 0;
                    end
                end
            end
        end

        % Original sigma-based reject/retry
        if ~skipSigmaReject && sig_hat > sigRejectK
            retry = retry + 1;
            if retry <= maxRetriesK
                continue;
            end
            sig_hat = min(cfg.sigmaCeil_deg, sig_hat * 1.3);
        end
        retry = 0;

        % accept vector
        measPos(k,:)   = p;
        measPsi(k)     = psi;
        measTh(k)      = th_hat_global;
        measSig(k)     = sig_hat;
        measSnr(k)     = snrMax_dB;
        measProm(k)    = prom_dB;
        measHalfW(k)   = halfW;
        measStep(k)    = stepD;
        measRange(k)   = range_from_power_ft(pmax_dB, cfg);

        % Robust rangeUse from recent accepted scans (median of last 2-3)
        i0 = max(1, k-2);
        ru = median(measRange(i0:k), 'omitnan');
        if ~isfinite(ru), ru = 0; end
        if k >= 2 && isfinite(measRange(k-1))
            rangeUse = 0.6*measRange(k-1) + 0.4*ru;
        else
            rangeUse = ru;
        end

        attemptsPerVec(k) = attemptsThisVec;
        attemptsThisVec = 0;
        acceptedCount = k;

                % PF update (bearing + bias + optional power)
        [pf_xyb, pf_w] = pf_update_with_bearing_bias( ...
            p, th_hat_global, sig_hat, pmaxForPF_dB, pf_xyb, pf_w, REGION_FT, pStart, cfg);

        % Camera update at accepted scan pose
        camBearing = th_hat_global;
        cameraSaw = target_in_camera_view(p, camBearing, tx, cfg);

        [pf_xyb, pf_w] = pf_update_with_camera(p, camBearing, cameraSaw, pf_xyb, pf_w, cfg, false);

        [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
        confR95Hist(k) = r95;

        if cameraSaw
            cameraFound = true;
            cameraHitPos = p;
            pred_pf = cameraHitPos;
            r95 = 0;
            confR95Hist(k) = 0;
            stopReason = "camera_found_scan";
            res.vec_used(t) = k;
            break;
        end

        [enterCamSearch, ~] = should_enter_camera_search(k, r95, p, pred_pf, pf_xyb, pf_w, timeUsed_s, cfg);
        if enterCamSearch && (cameraSearchAttempts < cfg.cameraSearchMaxAttempts)
            cameraSearchTriggered = true;
            cameraSearchAttempts = cameraSearchAttempts + 1;

            [p, psi, pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, travelTime_s, scanAttemptsTotal, ...
             stepCount, stepDists, segStart, segEnd, segType, segHit, cameraFound, cameraHitPos, searchFound, stopReason] = ...
                execute_camera_search_mode(p, psi, pred_pf, r95, tx, pf_xyb, pf_w, ...
                timeUsed_s, scanTime_s, travelTime_s, scanAttemptsTotal, stepCount, stepDists, ...
                segStart, segEnd, segType, segHit, cfg, REGION_FT, pStart);

            confR95Hist(k) = r95;
            if searchFound
                cameraSearchFound = true;
                res.vec_used(t) = k;
                break;
            else
                cameraSaw = false;
                camBearing = psi;
            end
        end

        % Final localization mode: switch planner params once close-ish
        timeLeft_s = cfg.timeBudget_s - timeUsed_s;
        finalMode = (isfinite(r95) && (r95 <= cfg.finalModeTrigger_r95_ft)) || ...
                    (timeLeft_s <= 90 && isfinite(r95) && r95 <= 140);

        % stop by confidence only if camera is not contradicting the lock
        if k >= cfg.stopConf_minVec && isfinite(r95) && (r95 <= cfg.stopConf_r95_ft)

            stopCamOK = true;
            if isfield(cfg,'stopRequireCameraConsistency') && cfg.stopRequireCameraConsistency
                stopCamOK = camera_stop_consistent( ...
                    p, camBearing, cameraSaw, pf_xyb(:,1:2), pf_w, cfg);
            end

            if stopCamOK
                [pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, stopConfirmed] = do_confirm_scans( ...
                    pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, ...
                    p, psi, tx, cfg, REGION_FT, pStart);

                if stopConfirmed && isfinite(r95) && (r95 <= cfg.stopConf_r95_ft)
                    res.vec_used(t) = k;
                    res.stop_by_conf(t) = true;
                    stopReason = "conf_r95";
                    break;
                end
            end
        end

                % Optional short probe move after first accepted vector
                        % Optional short probe move after first accepted vector
        if k == 1 && isfield(cfg,'firstProbeEnable') && cfg.firstProbeEnable
            doProbe = true;

            if isfield(cfg,'firstProbeRangeGate_ft') && isfinite(measRange(k))
                doProbe = (measRange(k) <= cfg.firstProbeRangeGate_ft);
            end

            if doProbe
                pProbe = p + cfg.firstProbe_ft * [cosd(measTh(k)), sind(measTh(k))];
                pProbe = clamp_to_region(pProbe, REGION_FT);
                psiProbe = wrap180(measTh(k));

                if hypot(pProbe(1)-p(1), pProbe(2)-p(2)) > 1e-6
                    [probeMoveDist_eff, probeMoveT] = move_time_with_turn(p, psi, pProbe, psiProbe, cfg);

                    if timeUsed_s + probeMoveT <= cfg.timeBudget_s
                        [hitCamProbe, tauCamProbe] = segment_camera_detection(p, psi, pProbe, psiProbe, tx, cfg);

                        if hitCamProbe
                            dLine_ft = hypot(pProbe(1)-p(1), pProbe(2)-p(2));
                            dPsi = abs(wrap180(psiProbe - psi));
                            turnCost_ft = cfg.turnRadius_ft * deg2rad(dPsi);

                            dToCam_ft = tauCamProbe * dLine_ft;
                            moveDist_eff_cam = turnCost_ft + dToCam_ft;
                            moveT_cam = moveDist_eff_cam * cfg.travel_sec_per_ft;

                            if timeUsed_s + moveT_cam <= cfg.timeBudget_s
                                timeUsed_s   = timeUsed_s + moveT_cam;
                                travelTime_s = travelTime_s + moveT_cam;

                                stepCount = stepCount + 1;
                                stepDists(stepCount) = moveDist_eff_cam;

                                pOld = p;
                                cameraHitPos = p + tauCamProbe * (pProbe - p);
                                p = cameraHitPos;
                                psi = wrap180(psi + tauCamProbe * wrap180(psiProbe - psi));
                                [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, "probe_move", true);

                                cameraFound = true;
                                pred_pf = cameraHitPos;
                                r95 = 0;
                                confR95Hist(k) = 0;
                                res.vec_used(t) = k;
                                stopReason = "camera_found_probe";
                                break;
                            end
                        else
                            timeUsed_s   = timeUsed_s + probeMoveT;
                            travelTime_s = travelTime_s + probeMoveT;

                            stepCount = stepCount + 1;
                            stepDists(stepCount) = probeMoveDist_eff;

                            pOld = p;
                            p = pProbe;
                            psi = psiProbe;
                            [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, "probe_move", false);

                            % Negative/positive camera update from probe pose
                            cameraSawProbe = target_in_camera_view(p, psi, tx, cfg);
                            [pf_xyb, pf_w] = pf_update_with_camera(p, psi, cameraSawProbe, pf_xyb, pf_w, cfg, finalMode);
                            [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
                            confR95Hist(k) = min_with_nan(confR95Hist(k), r95);

                            if cameraSawProbe
                                cameraFound = true;
                                cameraHitPos = p;
                                pred_pf = cameraHitPos;
                                r95 = 0;
                                confR95Hist(k) = 0;
                                res.vec_used(t) = k;
                                stopReason = "camera_found_probe";
                                break;
                            end
                        end
                    end
                end
            end
        end



        geomCooldown = max(0, geomCooldown - 1);

        % plan next pose
        if k == 1
            [pNew, psiNew] = plan_second_pose(p, psi, pred_pf, pf_xyb, pf_w, pStart, REGION_FT, cfg);
            minMove = 0.7 * cfg.minBaseline_ft;
            if hypot(pNew(1)-p(1), pNew(2)-p(2)) < minMove
                B = cfg.minBaseline_ft;
                pNew = p + B*[cosd(psi), sind(psi)];
                pNew = clamp_to_region(pNew, REGION_FT);
                psiNew = psi;
            end
        else
                        c = abs(wrap180(measTh(k) - measTh(k-1)));
            crossMin = min(c, 180-c);

            doGeomFix = false;
            Bgeom = 120;

            % Early stronger geometry fix right after the 2nd accepted vector
            if k == 2
                if crossMin < cfg.crossMinDeg_early
                    doGeomFix = true;
                    Bgeom = cfg.earlyGeomFixBaseline_ft;
                end
            elseif k >= 3
                if (crossMin < cfg.crossMinDeg_tail) && (geomCooldown == 0)
                    doGeomFix = true;
                    Bgeom = 120;
                end
            end

            if doGeomFix
                thAvg = ang_mean_deg(measTh(max(1,k-1):k));
                [pNew, psiNew] = plan_perp_pose(p, psi, thAvg, Bgeom, pStart, REGION_FT, cfg, pf_xyb, pf_w);
                geomCooldown = cfg.geomFixCooldownSteps;
            else
                [pNew, psiNew] = plan_next_pose_pf( ...
                    p, psi, pred_pf, pf_xyb, pf_w, measPos(1:k,:), measTh(1:k), ...
                    pStart, REGION_FT, cfg, finalMode, r95, rangeUse);

                minMove = 0.7 * cfg.minBaseline_ft;
                if hypot(pNew(1)-p(1), pNew(2)-p(2)) < minMove
                    B = cfg.minBaseline_ft;
                    pNew = p + B*[cosd(psi), sind(psi)];
                    pNew = clamp_to_region(pNew, REGION_FT);
                    psiNew = psi;
                end
            end
        end

        % time gating for move
        [moveDist_eff, moveT] = move_time_with_turn(p, psi, pNew, psiNew, cfg);

        % No in-place rotation allowed
        if hypot(pNew(1)-p(1), pNew(2)-p(2)) < 1e-6 && abs(wrap180(psiNew-psi)) > 1e-6
            stopReason = "blocked_in_place_turn";
            break;
        end

        if timeUsed_s + moveT > cfg.timeBudget_s
            stopReason = "time_before_move";
            break;
        end

                % Camera detection during move to next scan
        [hitCam, tauCam] = segment_camera_detection(p, psi, pNew, psiNew, tx, cfg);

        if hitCam
            dLine_ft = hypot(pNew(1)-p(1), pNew(2)-p(2));
            dPsi = abs(wrap180(psiNew - psi));
            turnCost_ft = cfg.turnRadius_ft * deg2rad(dPsi);

            dToCam_ft = tauCam * dLine_ft;
            moveDist_eff_cam = turnCost_ft + dToCam_ft;
            moveT_cam = moveDist_eff_cam * cfg.travel_sec_per_ft;

            if timeUsed_s + moveT_cam > cfg.timeBudget_s
                stopReason = "time_before_camera_hit";
                break;
            end

            timeUsed_s   = timeUsed_s + moveT_cam;
            travelTime_s = travelTime_s + moveT_cam;

            stepCount = stepCount + 1;
            stepDists(stepCount) = moveDist_eff_cam;

            pOld = p;
            cameraHitPos = p + tauCam * (pNew - p);
            p = cameraHitPos;
            psi = wrap180(psi + tauCam * wrap180(psiNew - psi));
            [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, "normal_move", true);

            cameraFound = true;
            pred_pf = cameraHitPos;
            r95 = 0;
            res.vec_used(t) = k;
            stopReason = "camera_found_move";
            break;
        else
            % Apply full move as before
            timeUsed_s   = timeUsed_s + moveT;
            travelTime_s = travelTime_s + moveT;

            stepCount = stepCount + 1;
            stepDists(stepCount) = moveDist_eff;

            pOld = p;
            p   = pNew;
            psi = wrap180(psiNew);
            [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, "normal_move", false);

            % Negative/positive camera update from new pose
            cameraSawMove = target_in_camera_view(p, psi, tx, cfg);
            [pf_xyb, pf_w] = pf_update_with_camera(p, psi, cameraSawMove, pf_xyb, pf_w, cfg, finalMode);
            [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
            confR95Hist(k) = min_with_nan(confR95Hist(k), r95);

            if cameraSawMove
                cameraFound = true;
                cameraHitPos = p;
                pred_pf = cameraHitPos;
                r95 = 0;
                confR95Hist(k) = 0;
                res.vec_used(t) = k;
                stopReason = "camera_found_move";
                break;
            end

            if k == MAXVEC
                res.vec_used(t) = MAXVEC;
                stopReason = "maxvec";
            end

            k = k + 1;
        end
    end

    if res.vec_used(t) == 0
        res.vec_used(t) = max(acceptedCount, 1);
    end

    % Snapshot the estimate exactly at the stop trigger, before any leftover-time end scans.
    if cameraFound
        pred_stop = cameraHitPos;
        r95_stop = 0;
    else
        pred_stop = pred_pf;
        if any(~isfinite(pred_stop))
            pred_stop = p;
        end
        pred_stop = clamp_point_to_annulus(pred_stop, pStart, cfg.rMin_ft, cfg.rMax_ft);
        pred_stop = clamp_to_region(pred_stop, REGION_FT);
        r95_stop = r95;
    end

    err_stop = hypot(pred_stop(1)-tx(1), pred_stop(2)-tx(2));
    res.stop_trigger_pred_x_ft(t) = pred_stop(1);
    res.stop_trigger_pred_y_ft(t) = pred_stop(2);
    res.stop_trigger_conf_r95_ft(t) = r95_stop;
    res.stop_trigger_err_ft(t) = err_stop;
    res.would_have_been_success_if_stopped_now(t)   = isfinite(err_stop) && err_stop <= THRESH_FT(1);
    res.would_have_been_within30_if_stopped_now(t)  = isfinite(err_stop) && err_stop <= THRESH_FT(2);
    res.would_have_been_within40_if_stopped_now(t)  = isfinite(err_stop) && err_stop <= THRESH_FT(3);
    res.would_have_been_within50_if_stopped_now(t)  = isfinite(err_stop) && err_stop <= THRESH_FT(4);
    res.would_have_been_within100_if_stopped_now(t) = isfinite(err_stop) && err_stop <= THRESH_FT(5);

    % If time remains at end, spend it on extra scans
    if ~cameraFound
        [pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal] = spend_remaining_time_scans( ...
            pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, p, psi, tx, cfg, REGION_FT, pStart);
    end

        % final estimate
    if cameraFound
        pred_final = cameraHitPos;
        r95 = 0;
    else
        pred_final = pred_pf;
        if any(~isfinite(pred_final))
            pred_final = p;
        end
        pred_final = clamp_point_to_annulus(pred_final, pStart, cfg.rMin_ft, cfg.rMax_ft);
        pred_final = clamp_to_region(pred_final, REGION_FT);
    end

    res.pred_start_dist_ft(t) = hypot(pred_final(1)-pStart(1), pred_final(2)-pStart(2));
    res.pred_x_ft(t) = pred_final(1);
    res.pred_y_ft(t) = pred_final(2);

    res.conf_r95_ft(t) = r95;
    res.stop_reason{t} = char(stopReason);

    err = hypot(pred_final(1)-tx(1), pred_final(2)-tx(2));
    res.final_err_ft(t) = err;

    res.success20(t)  = isfinite(err) && err <= THRESH_FT(1);
    res.success30(t)  = isfinite(err) && err <= THRESH_FT(2);
    res.success40(t)  = isfinite(err) && err <= THRESH_FT(3);
    res.success50(t)  = isfinite(err) && err <= THRESH_FT(4);
    res.success100(t) = isfinite(err) && err <= THRESH_FT(5);

    res.time_used_s(t)   = timeUsed_s;
    res.scan_time_s(t)   = scanTime_s;
    res.travel_time_s(t) = travelTime_s;

    res.move_total_ft(t) = nansum(stepDists(1:stepCount));
    if stepCount == 0
        res.move_steps_ft{t} = "";
    else
        res.move_steps_ft{t} = char(strjoin(string(stepDists(1:stepCount).'), "|"));
    end

    res.scan_attempts_total(t) = scanAttemptsTotal;
    if acceptedCount == 0
        res.scan_attempts_steps{t} = "";
    else
        res.scan_attempts_steps{t} = char(strjoin(string(attemptsPerVec(1:acceptedCount).'), "|"));
    end

    % store history (for plots)
    res.hist_pos{t}   = measPos(1:acceptedCount,:);
    res.hist_psi{t}   = measPsi(1:acceptedCount);
    res.hist_th{t}    = measTh(1:acceptedCount);
    res.hist_sig{t}   = measSig(1:acceptedCount);
    res.hist_snr{t}   = measSnr(1:acceptedCount);
    res.hist_prom{t}  = measProm(1:acceptedCount);
    res.hist_halfW{t} = measHalfW(1:acceptedCount);
    res.hist_step{t}  = measStep(1:acceptedCount);
    res.hist_r95{t}   = confR95Hist(1:acceptedCount);
    res.camera_found(t) = cameraFound;
    res.camera_search_triggered(t) = cameraSearchTriggered;
    res.camera_search_found(t) = cameraSearchFound;
    res.travel_seg_start{t} = segStart;
    res.travel_seg_end{t}   = segEnd;
    res.travel_seg_type{t}  = cellstr(segType);
    res.travel_seg_hit{t}   = segHit;
end
end

%% ===========================
% Sweep profile + Stage-2 policy
%% ===========================
function [halfW, stepD, tag] = select_sweep_profile(cfg, k, retry, acceptedCount, measSig, measSnr, measProm)

stepD = cfg.followSweepStep_deg;

if k == 1
    halfW = cfg.servoLimit_deg;
    stepD = cfg.firstSweepStep_deg;
    tag = "sweep240";
else
    halfW = min(60, cfg.servoLimit_deg);
    tag = "sweepFollow";

    if acceptedCount >= 2
        s1 = measSig(acceptedCount);
        s2 = measSig(acceptedCount-1);
        n1 = measSnr(acceptedCount);
        n2 = measSnr(acceptedCount-1);
        p1 = measProm(acceptedCount);
        p2 = measProm(acceptedCount-1);

        locked2 = isfinite(s1)&&isfinite(s2) && (s1 <= cfg.fineDisableSig_deg) && (s2 <= cfg.fineDisableSig_deg);
        good2   = isfinite(n1)&&isfinite(n2) && (n1 >= cfg.fineMinSnr_dB) && (n2 >= cfg.fineMinSnr_dB);
        prom2   = isfinite(p1)&&isfinite(p2) && (p1 >= cfg.fineDisableProm_dB) && (p2 >= cfg.fineDisableProm_dB);

        if locked2 && good2 && prom2
            halfW = max(30, min(halfW, 4*s1));
            stepD = 1;
            tag = "sweepNarrow";
        end
    end
end

if retry > 0
    halfW = min(cfg.servoLimit_deg, max(halfW, 90));
    stepD = max(stepD, 2);
    tag = "sweepReacq";
end
end

function cfgScan = make_scan_cfg(cfg, k, acceptedCount, measSig, measSnr, measProm)
cfgScan = cfg;

enableFine = true;
if k <= 2
    enableFine = true;
elseif acceptedCount >= 2
    s1 = measSig(acceptedCount);
    s2 = measSig(acceptedCount-1);
    n1 = measSnr(acceptedCount);
    n2 = measSnr(acceptedCount-1);
    p1 = measProm(acceptedCount);
    p2 = measProm(acceptedCount-1);

    locked = isfinite(s1)&&isfinite(s2) && (s1 <= cfg.fineDisableSig_deg) && (s2 <= cfg.fineDisableSig_deg);
    good   = isfinite(n1)&&isfinite(n2) && (n1 >= cfg.fineMinSnr_dB) && (n2 >= cfg.fineMinSnr_dB);
    prom   = isfinite(p1)&&isfinite(p2) && (p1 >= cfg.fineDisableProm_dB) && (p2 >= cfg.fineDisableProm_dB);

    enableFine = ~(locked && good && prom);
end

if ~enableFine
    cfgScan.fineHalfWidth_deg = 0;
    cfgScan.fineStep_deg      = 0;
end
end

function Ts = scan_time_seconds(halfW_deg, step_deg, cfg)
lim = cfg.servoLimit_deg;
lo = max(-halfW_deg, -lim);
hi = min(+halfW_deg, +lim);
nCoarse = floor((hi-lo)/step_deg) + 1;

fineHalf = cfg.fineHalfWidth_deg;
fineStep = cfg.fineStep_deg;
if fineHalf > 0 && fineStep > 0
    nFine = floor((2*fineHalf)/fineStep) + 1;
else
    nFine = 0;
end

Ts = (nCoarse + nFine) * cfg.dwell_s;
end

%% ===========================
% Scan model (returns prom)
%% ===========================
function [th_hat_global, sigma_deg, pmax_dB, snrMax_dB, prom_dB] = scan_bearing_from_power_heading( ...
    p_ft, psi_deg, tx_ft, centerGuess_rel_deg, halfW_deg, step_deg, cfg)

lim = cfg.servoLimit_deg;

navg     = cfg.samplesPerAngle;
fineHalf = cfg.fineHalfWidth_deg;
fineStep = cfg.fineStep_deg;

tau        = cfg.softmaxTau_dB;
minProm    = cfg.minPeakProm_dB;
ambInflate = cfg.ambSigmaInflate;

sigmaFloor = cfg.sigmaFloor_deg;
sigmaCeil  = cfg.sigmaCeil_deg;

noiseFloor_dB  = cfg.noiseFloor_dB;
noiseJitter_dB = cfg.noiseJitter_dB;

sigSat_dB  = cfg.sigSat_dB;
satDistK   = cfg.satDistK;
satInflate = cfg.satSigmaInflate;

snrMin_dB   = cfg.snrMin_dB;
snrMaxClamp = cfg.snrMax_dB;

th_true_global = atan2d(tx_ft(2)-p_ft(2), tx_ft(1)-p_ft(1));
th_true_rel    = wrap180(th_true_global - psi_deg);

d_ft = hypot(tx_ft(1)-p_ft(1), tx_ft(2)-p_ft(2));
d_m  = max(d_ft*0.3048, 0.05);
baseSig_dB = cfg.P0_dB - 10*cfg.plExp*log10(d_m / cfg.d0_m);

fade = cfg.fadeSigma_dB * randn;

% Stage 1
c  = centerGuess_rel_deg;
lo = max(c - halfW_deg, -lim);
hi = min(c + halfW_deg, +lim);
if ~isfinite(lo) || ~isfinite(hi) || step_deg <= 0 || hi < lo
    ths1 = max(min(c, lim), -lim);
else
    ths1 = lo:step_deg:hi;
end
if isempty(ths1), ths1 = max(min(c, lim), -lim); end

[snr1, sig1, over1] = measure_sweep_snr(ths1);
snr1 = snr1(:).'; sig1 = sig1(:).'; over1 = over1(:).'; ths1 = ths1(:).';

[snrMax1, i1] = max(snr1); if isempty(i1), i1 = 1; end
th_coarse = ths1(i1);
th_pick   = parabola_peak(ths1, snr1, i1);

% Stage 2 optional
 doFine = isfinite(fineHalf) && fineHalf > 0 && isfinite(fineStep) && fineStep > 0;
if doFine
    lo2 = max(th_coarse - fineHalf, -lim);
    hi2 = min(th_coarse + fineHalf, +lim);
    if hi2 < lo2, ths2 = th_coarse; else, ths2 = lo2:fineStep:hi2; end
    if isempty(ths2), ths2 = th_coarse; end

    [snr2, sig2, over2] = measure_sweep_snr(ths2);
    snr2 = snr2(:).'; sig2 = sig2(:).'; over2 = over2(:).'; ths2 = ths2(:).';

    [snrMax, i2] = max(snr2); if isempty(i2), i2 = 1; end
    th_pick = parabola_peak(ths2, snr2, i2);
    pmax_dB = sig2(i2);

    second = second_best_excluding_neighbors(snr2, i2, 1);
    prom_dB = snrMax - second;

    w = exp((snr2 - snrMax) / tau);
    sw = sum(w);
    if sw <= 0 || ~isfinite(sw)
        sigma_meas = sigmaCeil;
    else
        w = w / sw;
        dtheta = wrap180(ths2 - th_pick);
        mu = sum(w .* dtheta);
        varc = sum(w .* (dtheta - mu).^2);
        sigma_meas = sqrt(max(varc, 0));
    end

    sigma_quant = fineStep / sqrt(12);
    overUse = over2;
else
    snrMax  = snrMax1;
    pmax_dB = sig1(i1);
    overUse = over1;

    second = second_best_excluding_neighbors(snr1, i1, 1);
    prom_dB = snrMax - second;

    w = exp((snr1 - snrMax) / tau);
    sw = sum(w);
    if sw <= 0 || ~isfinite(sw)
        sigma_meas = sigmaCeil;
    else
        w = w / sw;
        dtheta = wrap180(ths1 - th_pick);
        mu = sum(w .* dtheta);
        varc = sum(w .* (dtheta - mu).^2);
        sigma_meas = sqrt(max(varc, 0));
    end

    sigma_quant = step_deg / sqrt(12);
end

sigma_deg = sqrt(cfg.servoSigma_deg^2 + sigma_meas^2 + sigma_quant^2);

if prom_dB < minProm
    sigma_deg = sigma_deg * ambInflate;
end

satFrac = mean(overUse > 1.0);
if satFrac > 0.05
    sigma_deg = sigma_deg * (1 + satInflate*satFrac);
end

sigma_deg = min(max(sigma_deg, sigmaFloor), sigmaCeil);

alpha_hat = wrap180(th_pick + cfg.servoSigma_deg*randn);
th_hat_global = wrap180(alpha_hat + psi_deg);

snrMax_dB = max(min(snrMax, snrMaxClamp), snrMin_dB);

    function [snr_dB, sigEff_dB, over] = measure_sweep_snr(ths)
        ths = ths(:).';
        off  = wrap180(ths - th_true_rel);
        gain = -12 * (off / cfg.HPBW_deg).^2;

        sigNoise = cfg.measSigma_dB * randn(navg, numel(ths));
        spikes   = (rand(navg, numel(ths)) < cfg.spikeProb) .* ...
                   (cfg.spikeMag_dB .* (2*(rand(navg, numel(ths))>0.5)-1));

        sig_dB = baseSig_dB + fade + gain + sigNoise + spikes;
        sig_dB = mean(sig_dB, 1);

        noise_dB = noiseFloor_dB + noiseJitter_dB*randn(1, numel(ths));

        Ps   = 10.^(sig_dB/10);
        Pn   = 10.^(noise_dB/10);
        Psat = 10.^(sigSat_dB/10);

        over = Ps ./ (Psat + 1e-12);
        Ps_eff = Ps ./ (1 + over);

        Pd = satDistK .* Pn .* (over.^2) ./ (1 + over).^2;
        Pn_eff = Pn + Pd;

        sigEff_dB = 10*log10(Ps_eff + 1e-12);
        snr_dB = 10*log10((Ps_eff ./ (Pn_eff + 1e-12)) + 1e-12);
        snr_dB = max(min(snr_dB, snrMaxClamp), snrMin_dB);
    end
end

%% ===========================
% PF with bias state: particles [x,y,b,pOff]
%% ===========================
function [xyb, w] = pf_init_particles_annulus_bias(pStart, REGION_FT, rMin, rMax, N, biasSigma0, powSigma0_dB)
xyb = zeros(N,4);
i = 1;
while i <= N
    r = rMin + (rMax-rMin)*rand;
    a = 2*pi*rand;
    cand = pStart + r*[cos(a), sin(a)];
    if all(cand >= 0) && all(cand <= REGION_FT)
        b    = biasSigma0   * randn;
        pOff = powSigma0_dB * randn;
        xyb(i,:) = [cand(1) cand(2) b pOff];
        i = i + 1;
    end
end
w = ones(N,1)/N;
end

function [xyb, w] = pf_update_with_bearing_bias(p, th_hat, sig_deg, p_meas_dB, xyb, w, REGION_FT, pStart, cfg)
% bearing likelihood
th = atan2d(xyb(:,2)-p(2), xyb(:,1)-p(1));
b  = xyb(:,3);
resid_th = wrap180((th + b) - th_hat);

sig = max(sig_deg, 0.5);
sig = max(cfg.pfSigmaScale * sig, 0.6);

g_th = exp(-0.5*(resid_th./sig).^2);
like_th = (1 - cfg.pfLikeEps)*g_th + cfg.pfLikeEps;
like_th = max(like_th, cfg.pfLikeFloor);

% power likelihood
like_p = ones(size(w));
if nargin >= 4 && isfinite(p_meas_dB)
    pOff = xyb(:,4);
    if isfield(cfg,'pfPowOffClamp_dB')
        pOff = max(min(pOff, cfg.pfPowOffClamp_dB), -cfg.pfPowOffClamp_dB);
        xyb(:,4) = pOff;
    end

    d_ft = hypot(xyb(:,1)-p(1), xyb(:,2)-p(2));
    d_m  = max(d_ft*0.3048, 0.05);

    p_pred = (cfg.P0_dB + pOff) - 10*cfg.plExp*log10(d_m / cfg.d0_m);

    rp = p_pred - p_meas_dB;
    if isfield(cfg,'pfPowResidClamp_dB')
        rp = max(min(rp, cfg.pfPowResidClamp_dB), -cfg.pfPowResidClamp_dB);
    end

    sigP = max(cfg.pfPowSigma_dB, 0.5);
    g_p = exp(-0.5*(rp./sigP).^2);
    epsP = cfg.pfPowLikeEps;
    like_p = (1 - epsP)*g_p + epsP;
    like_p = max(like_p, cfg.pfPowLikeFloor);
end

% combine + normalize
w = w .* like_th .* like_p;
sw = sum(w);
if ~isfinite(sw) || sw <= 0
    w = ones(size(w))/numel(w);
else
    w = w / sw;
end

% resample + jitter
ess = 1 / sum(w.^2);
if ess < cfg.pfResampleESSFrac * numel(w)
    [xyb, w] = pf_resample_systematic(xyb, w);

    xyb(:,1:2) = xyb(:,1:2) + cfg.pfJitter_ft * randn(size(xyb(:,1:2)));
    xyb(:,3)   = xyb(:,3)   + cfg.pfBiasJitter_deg * randn(size(xyb(:,3)));

    if isfield(cfg,'pfPowJitter_dB') && cfg.pfPowJitter_dB > 0
        xyb(:,4) = xyb(:,4) + cfg.pfPowJitter_dB * randn(size(xyb(:,4)));
    end

    xyb(:,1) = min(max(xyb(:,1),0),REGION_FT);
    xyb(:,2) = min(max(xyb(:,2),0),REGION_FT);

    xyb(:,1:2) = enforce_annulus_points(xyb(:,1:2), pStart, cfg.rMin_ft, cfg.rMax_ft, REGION_FT);

    if isfield(cfg,'pfPowOffClamp_dB')
        xyb(:,4) = max(min(xyb(:,4), cfg.pfPowOffClamp_dB), -cfg.pfPowOffClamp_dB);
    end
end
end

function pts = enforce_annulus_points(pts, pStart, rMin, rMax, REGION_FT)
N = size(pts,1);
for i = 1:N
    v = pts(i,:) - pStart;
    r = hypot(v(1), v(2));
    if ~isfinite(r) || r < 1e-9, r = 1e-9; end

    if r < rMin || r > rMax
        if r < rMin
            s = rMin/r;
        else
            s = rMax/r;
        end
        cand = pStart + s*v;
        cand = clamp_to_region(cand, REGION_FT);

        v2 = cand - pStart;
        r2 = hypot(v2(1), v2(2));
        if r2 < rMin || r2 > rMax
            ok = false;
            tries = 0;
            while ~ok && tries < 50
                rr = rMin + (rMax-rMin)*rand;
                aa = 2*pi*rand;
                cand2 = pStart + rr*[cos(aa), sin(aa)];
                ok = all(cand2>=0) && all(cand2<=REGION_FT);
                tries = tries + 1;
                if ok, cand = cand2; end
            end
        end
        pts(i,:) = cand;
    end
end
end

function [xy2, w2] = pf_resample_systematic(xy, w)
N = numel(w);
edges = cumsum(w);
edges(end) = 1;

u0 = rand/N;
u  = u0 + (0:N-1)'/N;

idx = zeros(N,1);
j = 1;
for i = 1:N
    while u(i) > edges(j)
        j = j + 1;
    end
    idx(i) = j;
end
xy2 = xy(idx,:);
w2 = ones(N,1)/N;
end

function [mode_xy, r95] = pf_estimate_radius95(xy, w)
[~, imax] = max(w);
c = xy(imax,:);

rad = 120;
d = hypot(xy(:,1)-c(1), xy(:,2)-c(2));
mask = d <= rad;

if sum(mask) >= 50
    ww = w(mask);
    ww = ww/sum(ww);
    xx = xy(mask,:);
    mode_xy = [sum(ww.*xx(:,1)), sum(ww.*xx(:,2))];
else
    mode_xy = c;
end

d2 = hypot(xy(:,1)-mode_xy(1), xy(:,2)-mode_xy(2));
r95 = weighted_quantile(d2, w, 0.95);
end

function q = weighted_quantile(x, w, p)
[xs, ord] = sort(x(:));
ws = w(ord);
cs = cumsum(ws);
k = find(cs >= p, 1, 'first');
if isempty(k)
    q = xs(end);
else
    q = xs(k);
end
end

%% ===========================
% Planner (pose = position + heading)
%% ===========================
function [p2, psi2] = plan_second_pose(p, psi, pred, pf_xyb, pf_w, pStart, REGION_FT, cfg)
baselines = [120 150 180 240 300];
offs = [+90 -90 +75 -75];

rStartHat = weighted_quantile(hypot(pf_xyb(:,1)-pStart(1), pf_xyb(:,2)-pStart(2)), pf_w, 0.50);
if ~isfinite(rStartHat), rStartHat = mean([cfg.rMin_ft cfg.rMax_ft]); end

bestScore = -inf;
p2 = p;
psi2 = psi;

if any(~isfinite(pred))
    th_hat = psi;
else
    th_hat = atan2d(pred(2)-p(2), pred(1)-p(1));
end

for B = baselines
    for o = offs
        cand = p + B*[cosd(th_hat+o), sind(th_hat+o)];
        if any(cand < 0) || any(cand > REGION_FT), continue; end

        rs = hypot(cand(1)-pStart(1), cand(2)-pStart(2));
        if rs > cfg.searchRmax_ft, continue; end

        ringPenalty = cfg.ringPenaltyW * abs(rs - rStartHat);

        psiCand = desired_heading_from_particles(cand, pf_xyb(:,1:2), pf_w);
        if ~isfinite(psiCand), continue; end

        unobs = unobservable_mass(cand, psiCand, pf_xyb(:,1:2), pf_w, cfg);
        [~, moveT] = move_time_with_turn(p, psi, cand, psiCand, cfg);

        score = -cfg.planMovePenalty*moveT ...
                -cfg.unobsPenalty*unobs ...
                -cfg.startRadiusPenalty*max(0, rs-cfg.rMax_ft) ...
                -ringPenalty;

        if score > bestScore
            bestScore = score;
            p2 = cand;
            psi2 = psiCand;
        end
    end
end
end

function [pNext, psiNext] = plan_next_pose_pf( ...
    pCur, psiCur, pred, pf_xyb, pf_w, prevPos, prevTh, pStart, REGION_FT, cfg, finalMode, r95, rangeUse)

if nargin < 13 || isempty(rangeUse) || ~isfinite(rangeUse), rangeUse = 0; end
if ~isfinite(r95), r95 = 0; end
if any(~isfinite(pred)), pred = pCur; end

if isfield(cfg,'planBearBins'),     nb = cfg.planBearBins;     else, nb = 24; end
if isfield(cfg,'planEntropyW'),     wEnt = cfg.planEntropyW;   else, wEnt = 2.0; end
if isfield(cfg,'planSplitW'),       wSplit = cfg.planSplitW;   else, wSplit = 1.2; end
if isfield(cfg,'planCircVarW'),     wCirc = cfg.planCircVarW;  else, wCirc = 0.6; end
if isfield(cfg,'planFarStepW'),     wFar = cfg.planFarStepW;   else, wFar = 0.35; end
if isfield(cfg,'minVisibleMass'),   minVis = cfg.minVisibleMass; else, minVis = 0.35; end
if isfield(cfg,'planFarRange_ft'),  farRange = cfg.planFarRange_ft; else, farRange = 320; end
if isfield(cfg,'planMaxOrbit_far_ft'), planMaxOrbitFar = cfg.planMaxOrbit_far_ft; else, planMaxOrbitFar = cfg.maxStep_ft; end

lim = cfg.servoLimit_deg - cfg.servoMargin_deg;
edges = linspace(-lim, +lim, nb+1);

P = pf_xyb(:,1:2);
w = pf_w(:);
sw = sum(w);
if sw <= 0 || ~isfinite(sw)
    w = ones(size(w))/numel(w);
else
    w = w/sw;
end

rStartHat = weighted_quantile(hypot(P(:,1)-pStart(1), P(:,2)-pStart(2)), w, 0.50);
if ~isfinite(rStartHat), rStartHat = mean([cfg.rMin_ft cfg.rMax_ft]); end

if finalMode
    planMaxOrbit = cfg.finalModeMaxOrbit_ft;
    radiiMult    = cfg.finalModeRadiiMult;
    maxStep      = min(cfg.finalModeMaxStep_ft, cfg.maxStep_ft);
else
    planMaxOrbit = cfg.planMaxOrbit_ft;
    radiiMult    = cfg.planRadiiMult;
    maxStep      = cfg.maxStep_ft;
end

if ~finalMode && isfinite(rangeUse) && (rangeUse >= farRange)
    planMaxOrbit = min(max(planMaxOrbit, planMaxOrbitFar), maxStep);
end

dPred = hypot(pred(1)-pCur(1), pred(2)-pCur(2));
if ~isfinite(dPred), dPred = 0; end

r0 = max([ ...
    0.6*dPred, ...
    0.7*r95, ...
    0.9*rangeUse, ...
    0.6*rStartHat ...
]);

if isfinite(rangeUse) && (rangeUse >= farRange)
    r0 = max(r0, 0.45*rangeUse);
end

r0 = min(max(r0, cfg.minBaseline_ft), planMaxOrbit);

angs = linspace(0, 360, cfg.planAngles+1);
angs(end) = [];

bestScore = -inf;
pNext = pCur;
psiNext = psiCur;

thLast  = prevTh(end);
thLast2 = prevTh(max(1,end-1));

for rm = radiiMult
    r = min(max(rm*r0, cfg.minBaseline_ft), maxStep);

    farBonus = 0;
    if isfinite(rangeUse) && rangeUse >= farRange
        farBonus = wFar * (r / max(planMaxOrbit,1));
    end

    for a = angs
        cand = pred + r*[cosd(a), sind(a)];
        if any(cand < 0) || any(cand > REGION_FT), continue; end

        rs = hypot(cand(1)-pStart(1), cand(2)-pStart(2));
        if rs > cfg.searchRmax_ft, continue; end

        step = hypot(cand(1)-pCur(1), cand(2)-pCur(2));
        if step > maxStep, continue; end

        psiCand = desired_heading_from_particles(cand, P, w);
        if ~isfinite(psiCand), continue; end

        unobs = unobservable_mass(cand, psiCand, P, w, cfg);
        visMass = 1 - unobs;
        if visMass < minVis, continue; end

        [~, moveT] = move_time_with_turn(pCur, psiCur, cand, psiCand, cfg);

        thp = atan2d(P(:,2)-cand(2), P(:,1)-cand(1));
        rel = wrap180(thp - psiCand);

        dCam = hypot(P(:,1)-cand(1), P(:,2)-cand(2));
        camVis = (dCam <= cfg.cameraDetect_ft) & (abs(rel) <= cfg.servoLimit_deg);
        camMass = sum(w(camVis));

        vis = abs(rel) <= lim;
        wv = w(vis);
        rv = rel(vis);

        if isempty(wv) || sum(wv) <= 0, continue; end

        leftMass = sum(wv(rv < 0));
        totMass  = sum(wv);
        splitScore = 1 - abs(leftMass - 0.5*totMass) / (0.5*totMass + 1e-12);

        bin = discretize(rv, edges);
        ok = ~isnan(bin);
        counts = accumarray(bin(ok), wv(ok), [nb 1], @sum, 0);
        pp = counts / max(sum(counts), 1e-12);
        nz = pp > 0;
        H = -sum(pp(nz).*log(pp(nz))) / log(nb);
        entScore = H;

        R = hypot(sum(wv.*cosd(rv)), sum(wv.*sind(rv)));
        circVar = 1 - R;

        infoScore = wEnt*entScore + wSplit*splitScore + wCirc*circVar;

        thNew = atan2d(pred(2)-cand(2), pred(1)-cand(1));
        c1 = abs(wrap180(thNew - thLast));  c1 = min(c1, 180-c1);
        c2 = abs(wrap180(thNew - thLast2)); c2 = min(c2, 180-c2);
        crossMin = min(c1, c2);
        crossScore = 1 - min(abs(90 - crossMin)/90, 1);

        dmin = min(hypot(prevPos(:,1)-cand(1), prevPos(:,2)-cand(2)));
        spacingScore = min(dmin, cfg.planSpacingCap_ft) / cfg.planSpacingCap_ft;

        ringPenalty = cfg.ringPenaltyW * abs(rs - rStartHat);

        score = ...
            + 2.2*infoScore ...
            + 1.2*crossScore ...
            + 1.0*spacingScore ...
            + farBonus ...
            - cfg.planMovePenalty*moveT ...
            - cfg.unobsPenalty*unobs ...
            - cfg.startRadiusPenalty*max(0, rs-cfg.rMax_ft) ...
            - ringPenalty;
        
        if finalMode
            score = score + 0.6*crossScore + cfg.finalCameraMassW * camMass;
        end

        if score > bestScore
            bestScore = score;
            pNext = cand;
            psiNext = psiCand;
        end
    end
end

if ~isfinite(bestScore) || bestScore == -inf
    B = max(cfg.minBaseline_ft, 120);
    pNext = pCur + B*[cosd(psiCur) sind(psiCur)];
    pNext = [min(max(pNext(1),0),REGION_FT), min(max(pNext(2),0),REGION_FT)];
    psiNext = psiCur;
end
end

function [p2, psi2] = plan_perp_pose(p1, psi1, th_deg, B, pStart, REGION_FT, cfg, pf_xyb, pf_w)

P = pf_xyb(:,1:2);
w = pf_w(:);
sw = sum(w);
if sw <= 0 || ~isfinite(sw)
    w = ones(size(w))/numel(w);
else
    w = w/sw;
end

if isfield(cfg,'planBearBins'), nb = cfg.planBearBins; else, nb = 24; end
if isfield(cfg,'planPerpEntropyW'), wEnt = cfg.planPerpEntropyW; else, wEnt = 1.6; end
if isfield(cfg,'planPerpSplitW'),   wSplit = cfg.planPerpSplitW; else, wSplit = 1.0; end
if isfield(cfg,'planPerpCrossW'),   wCross = cfg.planPerpCrossW; else, wCross = 1.2; end
if isfield(cfg,'minVisibleMass'),   minVis = cfg.minVisibleMass; else, minVis = 0.35; end

lim = cfg.servoLimit_deg - cfg.servoMargin_deg;
edges = linspace(-lim, +lim, nb+1);

rStartHat = weighted_quantile(hypot(P(:,1)-pStart(1), P(:,2)-pStart(2)), w, 0.50);
if ~isfinite(rStartHat), rStartHat = mean([cfg.rMin_ft cfg.rMax_ft]); end

vL = [cosd(th_deg+90), sind(th_deg+90)];
vR = [cosd(th_deg-90), sind(th_deg-90)];
cand = [p1 + B*vL; p1 + B*vR];

best = -inf;
p2 = p1;
psi2 = psi1;

th_from_p1 = atan2d(P(:,2)-p1(2), P(:,1)-p1(1));

for i = 1:2
    c = cand(i,:);
    if any(c < 0) || any(c > REGION_FT), continue; end

    rs = hypot(c(1)-pStart(1), c(2)-pStart(2));
    if rs > cfg.searchRmax_ft, continue; end

    psiCand0 = desired_heading_from_particles(c, P, w);
    if ~isfinite(psiCand0), continue; end

    psiCands = wrap180([psiCand0, psi1, psiCand0+30, psiCand0-30]);

    for psiCand = psiCands
        if ~isfinite(psiCand), continue; end

        unobs = unobservable_mass(c, psiCand, P, w, cfg);
        visMass = 1 - unobs;
        if visMass < minVis, continue; end

        [~, moveT] = move_time_with_turn(p1, psi1, c, psiCand, cfg);
        ringPenalty = cfg.ringPenaltyW * abs(rs - rStartHat);

        thp = atan2d(P(:,2)-c(2), P(:,1)-c(1));
        rel = wrap180(thp - psiCand);

        vis = abs(rel) <= lim;
        wv = w(vis);
        rv = rel(vis);
        if isempty(wv) || sum(wv) <= 0, continue; end

        tot = sum(wv);
        leftMass = sum(wv(rv < 0));
        splitScore = 1 - abs(leftMass - 0.5*tot)/(0.5*tot + 1e-12);

        bin = discretize(rv, edges);
        ok = ~isnan(bin);
        counts = accumarray(bin(ok), wv(ok), [nb 1], @sum, 0);
        pp = counts / max(sum(counts), 1e-12);
        nz = pp > 0;
        H = -sum(pp(nz).*log(pp(nz))) / log(nb);
        entScore = H;

        infoScore = wEnt*entScore + wSplit*splitScore;

        th_from_c = atan2d(P(:,2)-c(2), P(:,1)-c(1));
        cross = abs(wrap180(th_from_c - th_from_p1));
        cross = min(cross, 180-cross);
        err90 = abs(90 - cross);
        e25 = weighted_quantile(err90, w, 0.25);
        e50 = weighted_quantile(err90, w, 0.50);
        g25 = 1 - min(e25/90, 1);
        g50 = 1 - min(e50/90, 1);
        crossScore = 2.2*g25 + 0.8*g50;

        score = ...
            + 2.2*infoScore ...
            + wCross*crossScore ...
            - cfg.planMovePenalty*moveT ...
            - cfg.unobsPenalty*unobs ...
            - cfg.startRadiusPenalty*max(0, rs-cfg.rMax_ft) ...
            - ringPenalty;

        if score > best
            best = score;
            p2 = c;
            psi2 = psiCand;
        end
    end
end
end

function [score, psiCand, ok] = score_pose_candidate( ...
    cand, pCur, psiCur, crossRefPos, prevPosAll, P, w, pStart, cfg, ...
    wEnt, wSplit, wCirc, wCross, extraBonus)

if nargin < 14 || isempty(extraBonus)
    extraBonus = 0;
end

score   = -inf;
psiCand = psiCur;
ok      = false;

sw = sum(w);
if ~isfinite(sw) || sw <= 0
    return;
end
w = w / sw;

if isfield(cfg,'minVisibleMass'),   minVis = cfg.minVisibleMass; else, minVis = 0.35; end
if isfield(cfg,'planSpacingCap_ft'), spacingCap = cfg.planSpacingCap_ft; else, spacingCap = 300; end
if isfield(cfg,'planBearBins'),      nb = cfg.planBearBins; else, nb = 24; end

rs = hypot(cand(1)-pStart(1), cand(2)-pStart(2));
if rs > cfg.searchRmax_ft
    return;
end

psiCand = desired_heading_from_particles(cand, P, w);
if ~isfinite(psiCand)
    return;
end

unobs = unobservable_mass(cand, psiCand, P, w, cfg);
visMass = 1 - unobs;
if visMass < minVis
    return;
end

[~, moveT] = move_time_with_turn(pCur, psiCur, cand, psiCand, cfg);

lim   = cfg.servoLimit_deg - cfg.servoMargin_deg;
edges = linspace(-lim, +lim, nb+1);

thp = atan2d(P(:,2)-cand(2), P(:,1)-cand(1));
rel = wrap180(thp - psiCand);

vis = abs(rel) <= lim;
wv  = w(vis);
rv  = rel(vis);

if isempty(wv) || sum(wv) <= 0
    return;
end
wv = wv / sum(wv);

leftMass = sum(wv(rv < 0));
splitScore = 1 - abs(leftMass - 0.5) / 0.5;

bin = discretize(rv, edges);
okb = ~isnan(bin);
counts = accumarray(bin(okb), wv(okb), [nb 1], @sum, 0);
pp = counts / max(sum(counts), 1e-12);
nz = pp > 0;
entScore = -sum(pp(nz).*log(pp(nz))) / log(nb);

R = hypot(sum(wv.*cosd(rv)), sum(wv.*sind(rv)));
circVar = 1 - R;

infoScore = wEnt*entScore + wSplit*splitScore + wCirc*circVar;

if isempty(crossRefPos)
    crossScore = 0;
else
    if size(crossRefPos,2) ~= 2
        crossRefPos = reshape(crossRefPos, [], 2);
    end

    thCand = atan2d(P(:,2)-cand(2), P(:,1)-cand(1));
    crossVals = nan(size(crossRefPos,1),1);

    for j = 1:size(crossRefPos,1)
        thRef = atan2d(P(:,2)-crossRefPos(j,2), P(:,1)-crossRefPos(j,1));
        cross = abs(wrap180(thCand - thRef));
        cross = min(cross, 180-cross);

        err90 = abs(90 - cross);
        e25 = weighted_quantile(err90, w, 0.25);
        e50 = weighted_quantile(err90, w, 0.50);

        g25 = 1 - min(e25/90, 1);
        g50 = 1 - min(e50/90, 1);

        crossVals(j) = 2.2*g25 + 0.8*g50;
    end

    crossScore = min(crossVals);
end

if isempty(prevPosAll)
    spacingScore = 1.0;
else
    dmin = min(hypot(prevPosAll(:,1)-cand(1), prevPosAll(:,2)-cand(2)));
    spacingScore = min(dmin, spacingCap) / spacingCap;
end

rStartHat = weighted_quantile(hypot(P(:,1)-pStart(1), P(:,2)-pStart(2)), w, 0.50);
if ~isfinite(rStartHat)
    rStartHat = mean([cfg.rMin_ft cfg.rMax_ft]);
end
ringPenalty = cfg.ringPenaltyW * abs(rs - rStartHat);

score = ...
    + 2.2*infoScore ...
    + wCross*crossScore ...
    + 1.0*spacingScore ...
    + extraBonus ...
    - cfg.planMovePenalty*moveT ...
    - cfg.unobsPenalty*unobs ...
    - cfg.startRadiusPenalty*max(0, rs-cfg.rMax_ft) ...
    - ringPenalty;

ok = true;
end

function psiDes = desired_heading_from_particles(p, pf_xy, pf_w)
thp = atan2d(pf_xy(:,2)-p(2), pf_xy(:,1)-p(1));

m = isfinite(thp) & isfinite(pf_w) & (pf_w > 0);
if ~any(m)
    psiDes = 0;
    return;
end

w = pf_w(m);
th = thp(m);

sw = sum(w);
if ~isfinite(sw) || sw <= 0
    psiDes = atan2d(mean(sind(th)), mean(cosd(th)));
    return;
end

w = w / sw;
psiDes = atan2d(sum(w.*sind(th)), sum(w.*cosd(th)));

if ~isfinite(psiDes)
    psiDes = atan2d(mean(sind(th)), mean(cosd(th)));
end
end

function unobs = unobservable_mass(p, psi, pf_xy, pf_w, cfg)
thp = atan2d(pf_xy(:,2)-p(2), pf_xy(:,1)-p(1));
rel = wrap180(thp - psi);
lim = cfg.servoLimit_deg - cfg.servoMargin_deg;
mask = abs(rel) > lim;
unobs = sum(pf_w(mask));
end

function [moveDist_eff, moveT] = move_time_with_turn(p1, psi1, p2, psi2, cfg)
d = hypot(p2(1)-p1(1), p2(2)-p1(2));
dPsi = abs(wrap180(psi2 - psi1));
turnCost = cfg.turnRadius_ft * deg2rad(dPsi);
moveDist_eff = d + turnCost;
moveT = moveDist_eff * cfg.travel_sec_per_ft;
end

%% ===========================
% Extra scans
%% ===========================
function [pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, stopConfirmed] = do_confirm_scans( ...
    pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, p, psi, tx, cfg, REGION_FT, pStart)

stopConfirmed = true;

for j = 1:cfg.confirmScansOnStop
    halfW = min(cfg.extraScanHalfW_deg, cfg.servoLimit_deg);
    stepD = cfg.extraScanStep_deg;

    cfgScan = cfg;
    cfgScan.fineHalfWidth_deg = cfg.fineHalfWidth_deg;
    cfgScan.fineStep_deg      = cfg.fineStep_deg;

    scanT = scan_time_seconds(halfW, stepD, cfgScan);
    if timeUsed_s + scanT > cfg.timeBudget_s
        stopConfirmed = false;
        return;
    end

    timeUsed_s = timeUsed_s + scanT;
    scanTime_s = scanTime_s + scanT;
    scanAttemptsTotal = scanAttemptsTotal + 1;

    [th_hat_global, sig_hat, pmax_dB, ~, ~] = ...
        scan_bearing_from_power_heading(p, psi, tx, 0, halfW, stepD, cfgScan);

    [pf_xyb, pf_w] = pf_update_with_bearing_bias( ...
        p, th_hat_global, sig_hat, pmax_dB, pf_xyb, pf_w, REGION_FT, pStart, cfg);

    cameraSaw = target_in_camera_view(p, th_hat_global, tx, cfg);
    [pf_xyb, pf_w] = pf_update_with_camera(p, th_hat_global, cameraSaw, pf_xyb, pf_w, cfg, true);

    [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);

    if cameraSaw
        pred_pf = p;
        r95 = 0;
        return;
    end

    if ~(isfinite(r95) && (r95 <= cfg.stopConf_r95_ft))
        stopConfirmed = false;
        return;
    end

    if isfield(cfg,'stopRequireCameraConsistency') && cfg.stopRequireCameraConsistency
        if ~camera_stop_consistent(p, th_hat_global, cameraSaw, pf_xyb(:,1:2), pf_w, cfg)
            stopConfirmed = false;
            return;
        end
    end
end
end

function [pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal] = spend_remaining_time_scans( ...
    pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, scanAttemptsTotal, p, psi, tx, cfg, REGION_FT, pStart)

n = 0;
while n < cfg.extraEndScansMax
    halfW = min(cfg.extraScanHalfW_deg, cfg.servoLimit_deg);
    stepD = cfg.extraScanStep_deg;

    cfgScan = cfg;
    cfgScan.fineHalfWidth_deg = cfg.fineHalfWidth_deg;
    cfgScan.fineStep_deg      = cfg.fineStep_deg;

    scanT = scan_time_seconds(halfW, stepD, cfgScan);
    if timeUsed_s + scanT > cfg.timeBudget_s
        break;
    end

    timeUsed_s = timeUsed_s + scanT;
    scanTime_s = scanTime_s + scanT;
    scanAttemptsTotal = scanAttemptsTotal + 1;

    [th_hat_global, sig_hat, pmax_dB, ~, ~] = ...
        scan_bearing_from_power_heading(p, psi, tx, 0, halfW, stepD, cfgScan);

    [pf_xyb, pf_w] = pf_update_with_bearing_bias( ...
        p, th_hat_global, sig_hat, pmax_dB, pf_xyb, pf_w, REGION_FT, pStart, cfg);

    cameraSaw = target_in_camera_view(p, th_hat_global, tx, cfg);
    [pf_xyb, pf_w] = pf_update_with_camera(p, th_hat_global, cameraSaw, pf_xyb, pf_w, cfg, true);

    [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);

    n = n + 1;

    if cameraSaw
        pred_pf = p;
        r95 = 0;
        break;
    end

    if isfinite(r95) && r95 <= 12
        break;
    end
end
end

%% ===========================
% Plotting
%% ===========================
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
psi = res.hist_psi{tid}; %#ok<NASGU>
thh = res.hist_th{tid};
sig = res.hist_sig{tid};
segStart = []; segEnd = []; segType = {}; segHit = [];
if isfield(res,'travel_seg_start') && numel(res.travel_seg_start) >= tid && ~isempty(res.travel_seg_start{tid})
    segStart = res.travel_seg_start{tid};
    segEnd   = res.travel_seg_end{tid};
    segType  = res.travel_seg_type{tid};
    segHit   = res.travel_seg_hit{tid};
end

if isempty(pos) && isempty(segStart)
    title(sprintf("Trial %d (no scans?)", tid));
    return;
end

if ~isempty(segStart)
    draw_travel_segments(segStart, segEnd, segType, segHit);
elseif ~isempty(pos)
    plot(pos(:,1), pos(:,2), '-','LineWidth',2,'DisplayName','Travel');
end

for i = 1:size(pos,1)
    plot(pos(i,1),pos(i,2),'o','MarkerSize',10,'LineWidth',2,'DisplayName', ternary(i==1,"Scan locations",""));
    text(pos(i,1)+8, pos(i,2)+8, sprintf("%d", i), 'FontSize',11, 'FontWeight','bold');
end

if isempty(segStart)
    for i = 1:(size(pos,1)-1)
        dp = pos(i+1,:) - pos(i,:);
        quiver(pos(i,1),pos(i,2),dp(1),dp(2),0,'LineWidth',1.5,'MaxHeadSize',0.7,'DisplayName',ternary(i==1,"Travel direction",""));
    end
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

plotX = [pS(1); pT(1); pP(1)];
plotY = [pS(2); pT(2); pP(2)];
if ~isempty(pos), plotX = [plotX; pos(:,1)]; plotY = [plotY; pos(:,2)]; end
if ~isempty(segStart), plotX = [plotX; segStart(:,1); segEnd(:,1)]; plotY = [plotY; segStart(:,2); segEnd(:,2)]; end
xlim([max(0,min(plotX)-120) , min(REGION_FT,max(plotX)+120)]);
ylim([max(0,min(plotY)-120) , min(REGION_FT,max(plotY)+120)]);

err = res.final_err_ft(tid);
title(sprintf("Trial %d | err=%.1f ft | within20=%d | vec_used=%d | stop=%s", ...
    tid, err, res.success20(tid), res.vec_used(tid), string(res.stop_reason{tid})));

legend('Location','bestoutside');
xlabel('x (ft)'); ylabel('y (ft)');
end

%% ===========================
% Trial pickers
%% ===========================
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

%% ===========================
% Helpers
%% ===========================
function [tx, ok] = sample_target_annulus_in_region(pStart, REGION_FT, rMin, rMax)
r = rMin + (rMax-rMin)*rand;
a = 2*pi*rand;
tx = pStart + r*[cos(a), sin(a)];
ok = all(tx >= 0) && all(tx <= REGION_FT);
end

function p2 = clamp_to_region(p, REGION_FT)
p2 = [min(max(p(1),0),REGION_FT), min(max(p(2),0),REGION_FT)];
end

function p2 = clamp_point_to_annulus(p, pStart, rMin, rMax)
v = p - pStart;
r = hypot(v(1), v(2));
if ~isfinite(r) || r < 1e-9
    p2 = pStart + [rMin, 0];
    return;
end
if r < rMin
    p2 = pStart + (rMin/r)*v;
elseif r > rMax
    p2 = pStart + (rMax/r)*v;
else
    p2 = p;
end
end

function th = ang_mean_deg(ths)
ths = ths(:);
th = atan2d(mean(sind(ths)), mean(cosd(ths)));
end

function th = ang_mean_deg_weighted(ths, w)
ths = ths(:);
w = w(:) / sum(w);
th = atan2d(sum(w.*sind(ths)), sum(w.*cosd(ths)));
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function m = min_with_nan(a, b)
if ~isfinite(a)
    m = b;
elseif ~isfinite(b)
    m = a;
else
    m = min(a, b);
end
end

function a = wrap180(a)
a = mod(a + 180, 360) - 180;
end

function thp = parabola_peak(ths, y, i)
ths = ths(:);
y = y(:);
if ~isscalar(i)
    [~, i] = max(y);
end
i = double(i);
if i <= 1 || i >= numel(y)
    thp = ths(i);
    return;
end
x1 = ths(i-1); x2 = ths(i); x3 = ths(i+1);
y1 = y(i-1);   y2 = y(i);   y3 = y(i+1);
A = [x1^2 x1 1; x2^2 x2 1; x3^2 x3 1];
coef = A \ [y1; y2; y3];
aa = coef(1); bb = coef(2);
if abs(aa) < 1e-12
    thp = x2;
else
    thp = -bb/(2*aa);
end
end

function v2 = second_best_excluding_neighbors(y, imax, nb)
mask = true(size(y));
lo = max(1, imax-nb);
hi = min(numel(y), imax+nb);
mask(lo:hi) = false;
if any(mask)
    v2 = max(y(mask));
else
    v2 = max(y);
end
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

fprintf("\nScan attempts:\n");
fprintf("  median attempts (all trials): %.1f\n", median(res.scan_attempts_total));
fprintf("  median attempts (within20):   %.1f\n", median(res.scan_attempts_total(res.success20)));

denAll = max(res.vec_used, 1);
den20  = max(res.vec_used(res.success20), 1);
fprintf("  median attempts per accepted vector (all): %.2f\n", median(res.scan_attempts_total ./ denAll));
if any(res.success20)
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
fprintf("  median move_total_ft (within20):%.1f\n", median(res.move_total_ft(res.success20)));

fprintf("\nTime:\n");
fprintf("  median time_used_s (all):      %.1f\n", median(res.time_used_s));
fprintf("  median scan_time_s (all):      %.1f\n", median(res.scan_time_s));
fprintf("  median travel_time_s (all):    %.1f\n", median(res.travel_time_s));
fprintf("  median time_used_s (within20): %.1f\n", median(res.time_used_s(res.success20)));
end

function r_ft = range_from_power_ft(pmax_dB, cfg)
d_m = cfg.d0_m * 10.^((cfg.P0_dB - pmax_dB) / (10*cfg.plExp));
r_ft = max(d_m / 0.3048, 5);
end

function d = start_target_dist_ft(res)
d = hypot(res.tgt_x_ft - res.start_x_ft, res.tgt_y_ft - res.start_y_ft);
end

function tid = pick_success20_far_trial(res, exclude)
if nargin < 2, exclude = []; end
N = numel(res.final_err_ft);
allIdx = (1:N).';
d = start_target_dist_ft(res);

mask = res.success20 & ~ismember(allIdx, exclude(:));
idx = find(mask);

if isempty(idx)
    tid = [];
    return;
end

[~, j] = max(d(idx));
tid = idx(j);
end

function tid = pick_mid_small_spawn_trial(res, exclude)
if nargin < 2, exclude = []; end
N = numel(res.final_err_ft);
allIdx = (1:N).';
d = start_target_dist_ft(res);

mask = (res.final_err_ft > 50) & (res.final_err_ft <= 100) & ~ismember(allIdx, exclude(:));
idx = find(mask);

if isempty(idx)
    tid = [];
    return;
end

[~, j] = min(d(idx));
tid = idx(j);
end

function tid = pick_bad_small_spawn_trial(res, exclude)
if nargin < 2, exclude = []; end
N = numel(res.final_err_ft);
allIdx = (1:N).';
d = start_target_dist_ft(res);

mask = (res.final_err_ft > 100) & ~ismember(allIdx, exclude(:));
idx = find(mask);

if isempty(idx)
    tid = [];
    return;
end

[~, j] = min(d(idx));
tid = idx(j);
end

function tid = pick_far_within20_trial(res)
idx = find(res.success20);
if isempty(idx)
    tid = nan;
    return;
end

vals = [res.spawn_target_dist_ft(idx), abs(res.final_err_ft(idx) - 10)];
[~, ord] = sortrows(vals, [-1 2]);
tid = idx(ord(1));
end

function tid = pick_small_mid_trial(res)
idx = find(res.final_err_ft > 50 & res.final_err_ft <= 100);
if isempty(idx)
    tid = nan;
    return;
end

vals = [res.spawn_target_dist_ft(idx), abs(res.final_err_ft(idx) - 75)];
[~, ord] = sortrows(vals, [1 2]);
tid = idx(ord(1));
end

function tid = pick_small_bad_trial(res)
idx = find(res.final_err_ft > 100);
if isempty(idx)
    tid = nan;
    return;
end

vals = [res.spawn_target_dist_ft(idx), abs(res.final_err_ft(idx) - 150)];
[~, ord] = sortrows(vals, [1 2]);
tid = idx(ord(1));
end

function [enterSearch, estSearchTime_s] = should_enter_camera_search(k, r95, p, pred_pf, pf_xyb, pf_w, timeUsed_s, cfg)
enterSearch = false;
estSearchTime_s = inf;

if ~isfield(cfg,'cameraSearchEnable') || ~cfg.cameraSearchEnable
    return;
end
if ~isfinite(r95)
    return;
end

if any(~isfinite(pred_pf))
    [pred_pf, ~] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
end
if any(~isfinite(pred_pf))
    pred_pf = p;
end

dCenter = hypot(pred_pf(1)-p(1), pred_pf(2)-p(2));
d50 = weighted_quantile(hypot(pf_xyb(:,1)-p(1), pf_xyb(:,2)-p(2)), pf_w, 0.50);
if ~isfinite(d50)
    d50 = dCenter;
end

baseEligible = (k >= cfg.cameraSearchMinVec) && (r95 <= cfg.cameraSearchR95Trigger_ft);

satEligible = false;
if isfield(cfg,'rfSaturationEnable') && cfg.rfSaturationEnable
    satRange = min(dCenter, d50);
    satEligible = (k >= cfg.cameraSearchSatMinVec) && ...
                  (r95 <= cfg.cameraSearchSatRegionTrigger_ft) && ...
                  (satRange <= cfg.rfSaturationHandoff_ft) && ...
                  (satRange <= cfg.rfSaturationMax_ft);
end

if ~(baseEligible || satEligible)
    return;
end

timeLeft_s = cfg.timeBudget_s - timeUsed_s;
regionForSearch = r95;
if satEligible && ~baseEligible
    regionForSearch = min(r95, cfg.cameraSearchSatRegionTrigger_ft);
end
estSearchTime_s = estimate_camera_search_time(regionForSearch, cfg);
reqBuffer = cfg.cameraSearchTimeBuffer_s;
if satEligible && isfield(cfg,'cameraSearchSatTimeBuffer_s')
    reqBuffer = max(reqBuffer, cfg.cameraSearchSatTimeBuffer_s);
end
enterSearch = (estSearchTime_s <= (timeLeft_s - reqBuffer));
end

function estTime_s = estimate_camera_search_time(regionR_ft, cfg)
regionR_ft = max(regionR_ft, cfg.cameraSearchRegionFloor_ft);
area_ft2 = pi * regionR_ft^2;
coverRate = max(1e-6, (1/cfg.travel_sec_per_ft) * cfg.cameraSearchEffWidth_ft * cfg.cameraSearchCoverageEff);
guardTime = (numel(cfg.cameraSearchGuardAngles_deg) + 2) * cfg.dwell_s;
estTime_s = area_ft2 / coverRate + guardTime + 30;
end

function [p, psi, pf_xyb, pf_w, pred_pf, r95, timeUsed_s, scanTime_s, travelTime_s, scanAttemptsTotal, stepCount, stepDists, segStart, segEnd, segType, segHit, cameraFound, cameraHitPos, searchFound, stopReason] = ...
    execute_camera_search_mode(p, psi, pred_pf, r95, tx, pf_xyb, pf_w, timeUsed_s, scanTime_s, travelTime_s, scanAttemptsTotal, stepCount, stepDists, segStart, segEnd, segType, segHit, cfg, REGION_FT, pStart)

searchFound = false;
cameraFound = false;
cameraHitPos = [NaN NaN];
stopReason = "camera_search_return";

center = pred_pf;
if any(~isfinite(center))
    [center, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
end
center = clamp_point_to_annulus(center, pStart, cfg.rMin_ft, cfg.rMax_ft);
center = clamp_to_region(center, REGION_FT);

regionR = max(r95, cfg.cameraSearchRegionFloor_ft);
guardR  = max(cfg.cameraSearchGuardRadius_ft, 0.7*regionR);
v = p - center;
rv = hypot(v(1), v(2));
if rv < 1e-6
    v = [cosd(psi+180), sind(psi+180)];
else
    v = v / rv;
end
pGuard = center + guardR * v;
pGuard = clamp_to_region(pGuard, REGION_FT);
psiGuard = wrap180(atan2d(center(2)-pGuard(2), center(1)-pGuard(1)));

[p, psi, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, hitMove, camHitPos, stopTime] = ...
    execute_camera_move(p, psi, pGuard, psiGuard, tx, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, "rf_guard_move", cfg);
if stopTime
    stopReason = "time_before_camera_search";
    [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
    return;
end
if hitMove
    cameraFound = true;
    cameraHitPos = camHitPos;
    p = camHitPos;
    pred_pf = camHitPos;
    r95 = 0;
    stopReason = "camera_found_search_move";
    searchFound = true;
    return;
end

[guardPass, guardInfo, timeUsed_s, scanTime_s, scanAttemptsTotal] = rf_guard_check_quick(p, psiGuard, center, tx, timeUsed_s, scanTime_s, scanAttemptsTotal, cfg);
psi = psiGuard;

if ~guardPass
    [pf_xyb, pf_w] = pf_update_with_camera(p, psi, false, pf_xyb, pf_w, cfg, true);
    [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
    stopReason = "camera_search_guard_reject";
    return;
end

waypoints = build_camera_search_waypoints(center, regionR, cfg, REGION_FT);
for i = 1:size(waypoints,1)
    pNext = waypoints(i,:);
    if i < size(waypoints,1)
        dxy = waypoints(i+1,:) - pNext;
        psiNext = wrap180(atan2d(dxy(2), dxy(1)));
    else
        psiNext = wrap180(atan2d(center(2)-pNext(2), center(1)-pNext(1)));
    end

    [p, psi, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, hitMove, camHitPos, stopTime] = ...
        execute_camera_move(p, psi, pNext, psiNext, tx, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, "camera_search_move", cfg);
    if stopTime
        stopReason = "time_during_camera_search";
        break;
    end
    if hitMove
        cameraFound = true;
        cameraHitPos = camHitPos;
        pred_pf = camHitPos;
        r95 = 0;
        stopReason = "camera_found_search_pattern";
        searchFound = true;
        return;
    end

    cameraSaw = target_in_camera_view(p, psi, tx, cfg);
    [pf_xyb, pf_w] = pf_update_with_camera(p, psi, cameraSaw, pf_xyb, pf_w, cfg, true);
    [pred_pf, r95] = pf_estimate_radius95(pf_xyb(:,1:2), pf_w);
    if cameraSaw
        cameraFound = true;
        cameraHitPos = p;
        pred_pf = p;
        r95 = 0;
        stopReason = "camera_found_search_pose";
        searchFound = true;
        return;
    end
end

if guardInfo.centerLikely
    stopReason = "camera_search_complete";
else
    stopReason = "camera_search_guard_reject";
end
end

function [guardPass, info, timeUsed_s, scanTime_s, scanAttemptsTotal] = rf_guard_check_quick(p, psiCenter, center, tx, timeUsed_s, scanTime_s, scanAttemptsTotal, cfg)
info = struct('spread_dB', NaN, 'centerLikely', false, 'satLike', false);
angles = cfg.cameraSearchGuardAngles_deg(:).';
scanT = numel(angles) * cfg.dwell_s;
if timeUsed_s + scanT > cfg.timeBudget_s
    guardPass = false;
    return;
end
timeUsed_s = timeUsed_s + scanT;
scanTime_s = scanTime_s + scanT;
scanAttemptsTotal = scanAttemptsTotal + 1;

[pSig, pSnr] = coarse_signal_at_headings(p, psiCenter, tx, angles, cfg);
spread = max(pSig) - min(pSig);
centerIdx = find(angles == 0, 1, 'first');
if isempty(centerIdx), centerIdx = ceil(numel(angles)/2); end
centerLikely = pSig(centerIdx) >= (max(pSig) - cfg.cameraSearchGuardCenterSlack_dB);
satLike = (spread <= cfg.cameraSearchGuardSpreadMax_dB) && (mean(pSnr) >= cfg.cameraSearchGuardMinSnr_dB);

info.spread_dB = spread;
info.centerLikely = centerLikely;
info.satLike = satLike;
guardPass = satLike || centerLikely;
end

function [sigEff_dB, snr_dB] = coarse_signal_at_headings(p_ft, psiCenter_deg, tx_ft, relHeadings_deg, cfg)
navg = max(1, cfg.samplesPerAngle);
noiseFloor_dB  = cfg.noiseFloor_dB;
noiseJitter_dB = cfg.noiseJitter_dB;
sigSat_dB  = cfg.sigSat_dB;
satDistK   = cfg.satDistK;
snrMin_dB   = cfg.snrMin_dB;
snrMaxClamp = cfg.snrMax_dB;

th_true_global = atan2d(tx_ft(2)-p_ft(2), tx_ft(1)-p_ft(1));
th_true_rel_to_center = wrap180(th_true_global - psiCenter_deg);

d_ft = hypot(tx_ft(1)-p_ft(1), tx_ft(2)-p_ft(2));
d_m  = max(d_ft*0.3048, 0.05);
baseSig_dB = cfg.P0_dB - 10*cfg.plExp*log10(d_m / cfg.d0_m);
fade = cfg.fadeSigma_dB * randn;

relHeadings_deg = relHeadings_deg(:).';
off = wrap180(relHeadings_deg - th_true_rel_to_center);
gain = -12 * (off / cfg.HPBW_deg).^2;

sigNoise = cfg.measSigma_dB * randn(navg, numel(relHeadings_deg));
spikes = (rand(navg, numel(relHeadings_deg)) < cfg.spikeProb) .* (cfg.spikeMag_dB .* (2*(rand(navg, numel(relHeadings_deg))>0.5)-1));
sig_dB = baseSig_dB + fade + gain + sigNoise + spikes;
sig_dB = mean(sig_dB, 1);
noise_dB = noiseFloor_dB + noiseJitter_dB*randn(1, numel(relHeadings_deg));
Ps   = 10.^(sig_dB/10);
Pn   = 10.^(noise_dB/10);
Psat = 10.^(sigSat_dB/10);
over = Ps ./ (Psat + 1e-12);
Ps_eff = Ps ./ (1 + over);
Pd = satDistK .* Pn .* (over.^2) ./ (1 + over).^2;
Pn_eff = Pn + Pd;
sigEff_dB = 10*log10(Ps_eff + 1e-12);
snr_dB = 10*log10((Ps_eff ./ (Pn_eff + 1e-12)) + 1e-12);
snr_dB = max(min(snr_dB, snrMaxClamp), snrMin_dB);
end

function waypoints = build_camera_search_waypoints(center, regionR, cfg, REGION_FT)
span = max(regionR, cfg.cameraSearchRegionFloor_ft);
spacing = max(5, cfg.cameraSearchSpacing_ft);
ys = -span:spacing:span;
if isempty(ys) || ys(end) < span
    ys = [ys, span];
end
xs = -span:spacing:span;
if isempty(xs) || xs(end) < span
    xs = [xs, span];
end
waypoints = [];
flipDir = false;
for iy = 1:numel(ys)
    y = center(2) + ys(iy);
    if flipDir
        xrow = fliplr(center(1) + xs);
    else
        xrow = center(1) + xs;
    end
    pts = [xrow(:), y*ones(numel(xrow),1)];
    pts(:,1) = min(max(pts(:,1),0),REGION_FT);
    pts(:,2) = min(max(pts(:,2),0),REGION_FT);
    waypoints = [waypoints; pts]; %#ok<AGROW>
    flipDir = ~flipDir;
end
if ~isempty(waypoints)
    d = hypot(waypoints(:,1)-center(1), waypoints(:,2)-center(2));
    [~, ord] = sort(d, 'ascend');
    first = waypoints(ord(1),:);
    [~, firstIdx] = min(hypot(waypoints(:,1)-first(1), waypoints(:,2)-first(2)));
    waypoints = [waypoints(firstIdx:end,:); waypoints(1:firstIdx-1,:)];
end
end

function [p, psi, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, hitCam, cameraHitPos, timeBlocked] = execute_camera_move(p, psi, pNext, psiNext, tx, timeUsed_s, travelTime_s, stepCount, stepDists, segStart, segEnd, segType, segHit, moveType, cfg)
[moveDist_eff, moveT] = move_time_with_turn(p, psi, pNext, psiNext, cfg);
timeBlocked = false;
hitCam = false;
cameraHitPos = [NaN NaN];
if timeUsed_s + moveT > cfg.timeBudget_s
    timeBlocked = true;
    return;
end
[hitCamSeg, tauCam] = segment_camera_detection(p, psi, pNext, psiNext, tx, cfg);
if hitCamSeg
    dLine_ft = hypot(pNext(1)-p(1), pNext(2)-p(2));
    dPsi = abs(wrap180(psiNext - psi));
    turnCost_ft = cfg.turnRadius_ft * deg2rad(dPsi);
    dToCam_ft = tauCam * dLine_ft;
    moveDist_eff_cam = turnCost_ft + dToCam_ft;
    moveT_cam = moveDist_eff_cam * cfg.travel_sec_per_ft;
    if timeUsed_s + moveT_cam > cfg.timeBudget_s
        timeBlocked = true;
        return;
    end
    timeUsed_s = timeUsed_s + moveT_cam;
    travelTime_s = travelTime_s + moveT_cam;
    stepCount = stepCount + 1;
    stepDists(stepCount) = moveDist_eff_cam;
    pOld = p;
    cameraHitPos = p + tauCam * (pNext - p);
    p = cameraHitPos;
    psi = wrap180(psi + tauCam * wrap180(psiNext - psi));
    [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, moveType, true);
    hitCam = true;
else
    timeUsed_s = timeUsed_s + moveT;
    travelTime_s = travelTime_s + moveT;
    stepCount = stepCount + 1;
    stepDists(stepCount) = moveDist_eff;
    pOld = p;
    p = pNext;
    psi = wrap180(psiNext);
    [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, pOld, p, moveType, false);
end
end

function [segStart, segEnd, segType, segHit] = append_travel_segment(segStart, segEnd, segType, segHit, p1, p2, typeStr, hitFlag)
if any(~isfinite([p1(:); p2(:)]))
    return;
end
if hypot(p2(1)-p1(1), p2(2)-p1(2)) < 1e-9
    return;
end
segStart(end+1,:) = p1; %#ok<AGROW>
segEnd(end+1,:)   = p2; %#ok<AGROW>
segType(end+1,1)  = string(typeStr); %#ok<AGROW>
segHit(end+1,1)   = logical(hitFlag); %#ok<AGROW>
end

function draw_travel_segments(segStart, segEnd, segType, segHit)
seen = struct('normal',false,'probe',false,'guard',false,'search',false,'hit',false);
for i = 1:size(segStart,1)
    p1 = segStart(i,:);
    p2 = segEnd(i,:);
    t = string(segType{i});
    if t == "normal_move"
        lbl = ternary(~seen.normal, "Normal move", "");
        plot([p1(1) p2(1)], [p1(2) p2(2)], '-', 'LineWidth', 1.8, 'Color', [0.10 0.10 0.10], 'DisplayName', lbl);
        seen.normal = true;
    elseif t == "probe_move"
        lbl = ternary(~seen.probe, "Probe move", "");
        plot([p1(1) p2(1)], [p1(2) p2(2)], '--', 'LineWidth', 2.0, 'Color', [0.85 0.33 0.10], 'DisplayName', lbl);
        seen.probe = true;
    elseif t == "rf_guard_move"
        lbl = ternary(~seen.guard, "RF guard move", "");
        plot([p1(1) p2(1)], [p1(2) p2(2)], '-.', 'LineWidth', 2.0, 'Color', [0.49 0.18 0.56], 'DisplayName', lbl);
        seen.guard = true;
    elseif t == "camera_search_move"
        lbl = ternary(~seen.search, "Camera search move", "");
        plot([p1(1) p2(1)], [p1(2) p2(2)], '-', 'LineWidth', 2.2, 'Color', [0.00 0.45 0.74], 'DisplayName', lbl);
        seen.search = true;
    else
        plot([p1(1) p2(1)], [p1(2) p2(2)], '-', 'LineWidth', 1.5, 'Color', [0.30 0.30 0.30]);
    end
    if i <= numel(segHit) && segHit(i)
        lbl = ternary(~seen.hit, "Camera hit during move", "");
        plot(p2(1), p2(2), 's', 'MarkerSize', 7, 'LineWidth', 1.5, 'Color', [0.85 0.00 0.00], 'DisplayName', lbl);
        seen.hit = true;
    end
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

function vis = target_in_camera_view(p, camBearing_deg, tx, cfg)
d = hypot(tx(1)-p(1), tx(2)-p(2));
if d > cfg.cameraDetect_ft
    vis = false;
    return;
end

th = atan2d(tx(2)-p(2), tx(1)-p(1));
rel = wrap180(th - camBearing_deg);

vis = abs(rel) <= cfg.servoLimit_deg;
end

function vis = particles_in_camera_view(p, camBearing_deg, pf_xy, cfg)
d = hypot(pf_xy(:,1)-p(1), pf_xy(:,2)-p(2));
th = atan2d(pf_xy(:,2)-p(2), pf_xy(:,1)-p(1));
rel = wrap180(th - camBearing_deg);

vis = (d <= cfg.cameraDetect_ft) & (abs(rel) <= cfg.servoLimit_deg);
end

function wCam = camera_negative_weight_from_pf(p, pf_xyb, pf_w, cfg, finalMode)
d = hypot(pf_xyb(:,1)-p(1), pf_xyb(:,2)-p(2));
d50 = weighted_quantile(d, pf_w, 0.50);

if nargin >= 5 && finalMode
    wCam = cfg.cameraNegW_close;
elseif isfinite(d50) && d50 <= cfg.cameraNegClose_ft
    wCam = cfg.cameraNegW_close;
elseif isfinite(d50) && d50 <= cfg.cameraNegMid_ft
    wCam = cfg.cameraNegW_mid;
else
    wCam = cfg.cameraNegW_far;
end
end

function [pf_xyb, pf_w] = pf_update_with_camera(p, camBearing_deg, cameraSawTarget, pf_xyb, pf_w, cfg, finalMode)
if nargin < 7
    finalMode = false;
end

vis = particles_in_camera_view(p, camBearing_deg, pf_xyb(:,1:2), cfg);
like = ones(size(pf_w));

if cameraSawTarget
    like(vis)  = cfg.cameraDetectProb;
    like(~vis) = max(cfg.cameraFalseAlarm, 1e-6);
else
    if isfield(cfg,'cameraUseNegative') && cfg.cameraUseNegative
        like(vis)  = cfg.cameraMissProb;
        like(~vis) = 1.0;

        wCam = camera_negative_weight_from_pf(p, pf_xyb, pf_w, cfg, finalMode);
        like = 1 + wCam * (like - 1);
    end
end

pf_w = pf_w .* like;
sw = sum(pf_w);

if ~isfinite(sw) || sw <= 0
    pf_w = ones(size(pf_w)) / numel(pf_w);
else
    pf_w = pf_w / sw;
end
end

function [hit, tauHit] = segment_camera_detection(p1, psi1, p2, psi2, tx, cfg)
d = hypot(p2(1)-p1(1), p2(2)-p1(2));
n = max(2, ceil(d / max(cfg.cameraSampleStep_ft, 0.5)));

hit = false;
tauHit = NaN;

dpsi = wrap180(psi2 - psi1);

for i = 0:n
    tau = i / n;
    p = p1 + tau * (p2 - p1);
    psi = wrap180(psi1 + tau * dpsi);

    if target_in_camera_view(p, psi, tx, cfg)
        hit = true;
        tauHit = tau;
        return;
    end
end
end

function [hit, tau] = segment_enters_radius(p1, p2, c, r)
% Returns whether the line segment p1->p2 enters the circle centered at c
% with radius r. tau is the FIRST entry fraction in [0,1].

d = p2 - p1;
f = p1 - c;

if dot(f,f) <= r^2
    hit = true;
    tau = 0;
    return;
end

a = dot(d,d);
if a < 1e-12
    hit = false;
    tau = NaN;
    return;
end

b = 2*dot(f,d);
cc = dot(f,f) - r^2;

disc = b^2 - 4*a*cc;
if disc < 0
    hit = false;
    tau = NaN;
    return;
end

sdisc = sqrt(disc);
t1 = (-b - sdisc) / (2*a);
t2 = (-b + sdisc) / (2*a);

ts = sort([t1 t2]);
ts = ts(ts >= 0 & ts <= 1);

if isempty(ts)
    hit = false;
    tau = NaN;
else
    hit = true;
    tau = ts(1);
end
end

function ok = camera_stop_consistent(p, camBearing_deg, cameraSaw, pf_xy, pf_w, cfg)

if cameraSaw
    ok = true;
    return;
end

w = pf_w(:);
sw = sum(w);
if ~isfinite(sw) || sw <= 0
    ok = true;
    return;
end
w = w / sw;

vis = particles_in_camera_view(p, camBearing_deg, pf_xy, cfg);
visMass = sum(w(vis));

ok = (visMass <= cfg.stopCameraVisibleMassMax);
end
