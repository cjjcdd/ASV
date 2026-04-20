"""
timesfm_sysid_patch.py
======================
TimeFM-enhanced System Identification for the ASV Digital Twin.
Includes Adaptive Sign Detection to handle inverted sensor conventions (e.g. Test 3).
"""

import numpy as np
import pandas as pd
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import uniform_filter1d
from scipy.optimize import minimize

try:
    import torch
    import timesfm
    _TIMESFM_AVAILABLE = True
except ImportError:
    _TIMESFM_AVAILABLE = False

# Global TimeFM instance to prevent memory lockups on rapid reloads
_TF_MODEL = None

def _get_timesfm_model():
    global _TF_MODEL
    if not _TIMESFM_AVAILABLE:
        return None
    if _TF_MODEL is None:
        try:
            torch.set_float32_matmul_precision("high")
            # Load your specific 200M parameter model
            _TF_MODEL = timesfm.TimesFM_2p5_200M_torch.from_pretrained("google/timesfm-2.5-200m-pytorch")
            
            # Compile with exact configurations provided in your example
            _TF_MODEL.compile(
                timesfm.ForecastConfig(
                    max_context=1024,
                    max_horizon=256,
                    normalize_inputs=True,
                    use_continuous_quantile_head=True,
                    force_flip_invariance=True,
                    infer_is_positive=True,
                    fix_quantile_crossing=True,
                )
            )
        except Exception as e:
            print(f"[TimeFM] Load failed, falling back to scipy filter: {e}")
            return None
    return _TF_MODEL

def _timesfm_smooth(signal_array, horizon=12):
    """Zero-shot denoising using TimeFM 1-step-ahead forecasting."""
    model = _get_timesfm_model()
    if model is None:
        return uniform_filter1d(signal_array, size=21)
    
    n = len(signal_array)
    smoothed = np.copy(signal_array)
    
    # Process in overlapping chunks
    step = horizon
    # Ensure minimum context of 32 for stability
    context_len = min(512, max(32, n // 4)) 
    
    for i in range(context_len, n, step):
        ctx = signal_array[i-context_len:i]
        try:
            # We only need the point_forecast (index 0 of the tuple)
            point_forecast, _ = model.forecast(horizon=horizon, inputs=[ctx])
            end_idx = min(i + horizon, n)
            chunk_len = end_idx - i
            # Output is shape (1, horizon). Take first batch.
            smoothed[i:end_idx] = point_forecast[0][:chunk_len]
        except Exception as e:
            pass # Fallback to raw data if an edge case fails
            
    return smoothed


class TimeFMSysIDWorker(QThread):
    """Drop-in QThread replacement for SysIDWorker."""
    # Matches the exact argument signature expected by ros_gui.py
    # _on_sysid_done(self, k_coeffs, t_coeffs, r_bias, mse)
    finished = pyqtSignal(list, list, float, float) 
    progress = pyqtSignal(str)
    
    def __init__(self, val_data, parent=None):
        super().__init__(parent)
        self.val_data = val_data

    def run(self):
        try:
            self.progress.emit("Stage 1: Denoising sensor arrays with TimeFM...")
            t = np.array(self.val_data['t'], dtype=float)
            r_raw = np.array(self.val_data['r'], dtype=float)
            u = np.array(self.val_data['u'], dtype=float)
            delta = np.array(self.val_data['delta'], dtype=float)
            udot = np.array(self.val_data['u_dot'], dtype=float)
            
            dt = float(np.median(np.diff(t)))
            
            # Smooth signals
            r_c = _timesfm_smooth(r_raw)
            u_c = uniform_filter1d(u, size=11)
            
            # Dynamic Bias Removal
            bias_mask = (np.abs(delta) < 1.0)
            r_bias = float(np.median(r_c[bias_mask])) if bias_mask.sum() > 10 else float(np.median(r_c))
            r_db = r_c - r_bias

            self.progress.emit("Stage 2: Physics-informed Windowed OLS...")
            
            # --- CRITICAL FIX: ADAPTIVE PHYSICS SIGN ---
            # Detects if the dataset has an inverted sensor convention (like Test 3)
            active = np.abs(delta) > 5.0
            if active.sum() > 0:
                corr = float(np.corrcoef(delta[active], r_db[active])[0,1])
                k_sign = 1 if corr > 0 else -1
            else:
                k_sign = 1
                
            # If the dataset is inverted, temporarily align delta with r to prevent 
            # the bounds rejection logic from throwing away valid windows
            delta_eff = delta * k_sign
            r_dot = np.gradient(r_db, dt)
            delta_rad = np.radians(delta_eff)
            
            K_list, T_list, feat_list = [], [], []
            N = len(t)
            
            for i in range(0, N - 60, 10):
                s = slice(i, i + 60)
                if np.abs(delta_eff[s]).max() < 2.0: continue  
                
                A = np.column_stack([r_dot[s], -delta_rad[s]])
                b = -r_db[s]
                try:
                    cond = np.linalg.cond(A)
                    if not np.isfinite(cond) or cond > 1e8: continue
                    
                    x, _, _, _ = np.linalg.lstsq(A, b, rcond=1e-10)
                    T_l, K_l = float(x[0]), float(x[1])
                    
                    # Because we flipped delta_eff above, K_l should now always be positive
                    if 0.001 < T_l < 2.0 and 0.001 < K_l < 300.0:
                        K_list.append(K_l)
                        T_list.append(T_l)
                        feat_list.append([np.mean(np.abs(delta[s])), np.mean(np.abs(u_c[s])), np.mean(np.abs(udot[s]))])
                except: pass

            self.progress.emit("Stage 3: TimeFM Matrix Ridge Fit...")
            if not K_list:
                raise ValueError("Insufficient rudder excitation in CSV. Please provide data with >2deg delta.")
                
            K_a, T_a, F_a = np.array(K_list), np.array(T_list), np.array(feat_list)
            
            K_a = _timesfm_smooth(K_a)
            T_a = _timesfm_smooth(T_a)
            
            F_mat = np.column_stack([np.ones(len(F_a)), F_a])
            lam = 0.1
            FtF = F_mat.T @ F_mat + lam * np.eye(4)
            k_seed = np.linalg.solve(FtF, F_mat.T @ K_a).tolist()
            t_seed = np.linalg.solve(FtF, F_mat.T @ T_a).tolist()

            self.progress.emit("Stage 4: L-BFGS-B Global Polishing...")
            d_abs, u_abs, ud_abs = np.abs(delta_eff), np.abs(u_c), np.abs(udot)
            
            bounds = [
                (0.001, 300.0), (0.0, 20.0), (0.0, 20.0), (0.0, 50.0),
                (0.001, 2.0),   (0.0, 0.5),  (0.0, 0.5),  (0.0, 3.0),
            ]
            
            def obj(p):
                aK, bK, cK, dK, aT, bT, cT, dT = p
                K_arr = aK + bK*d_abs + cK*u_abs + dK*ud_abs
                T_arr = np.maximum(aT + bT*d_abs + cT*u_abs + dT*ud_abs, 1e-3)
                e_arr = np.exp(-dt/T_arr)
                
                # --- CRITICAL FIX: Catch mid-turn initial states ---
                r_s = float(r_db[0]) 
                err = 0.0
                for i in range(1, N):
                    r_s = r_s*e_arr[i] + K_arr[i]*delta_rad[i]*(1-e_arr[i])
                    err += (r_s - r_db[i])**2
                return err / N

            K_med = np.median(K_list)
            T_med = np.median(T_list)
            starts = [
                k_seed + t_seed,
                [K_med, 0, 0, 5, T_med, 0, 0, 0.5],
                [50, 2, 3, 5, 0.05, 0.02, 0.02, 0.2]
            ]
            
            best_res = None
            for x0 in starts:
                x0_clipped = [float(np.clip(v, bounds[i][0], bounds[i][1])) for i,v in enumerate(x0)]
                try:
                    res = minimize(obj, x0_clipped, method='L-BFGS-B', bounds=bounds, options={'maxiter':500, 'ftol':1e-12})
                    if best_res is None or res.fun < best_res.fun:
                        best_res = res
                except: pass

            # Unpack optimal coefficients
            final_k = [float(v) for v in best_res.x[:4]]
            final_t = [float(v) for v in best_res.x[4:8]]
            
            # Re-apply the physical sign flip if the dataset was inverted
            # We multiply all K params by k_sign so the equation stays balanced 
            # in the dynamics node.
            final_k = [v * k_sign for v in final_k]
            final_mse = float(best_res.fun)

            self.progress.emit("SysID Complete!")
            self.finished.emit(final_k, final_t, r_bias, final_mse)

        except Exception as e:
            self.progress.emit(f"Failed: {str(e)}")
            self.finished.emit([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], 0.0, 0.0)