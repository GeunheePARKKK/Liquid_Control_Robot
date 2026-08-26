import argparse
import glob
import numpy as np
import pandas as pd
from scipy.signal import welch

FS = 100          # ESP32 로깅 샘플링 주파수 (esp32_imu_logger.ino의 SAMPLE_INTERVAL_MS=10 -> 100Hz)
WINDOW_SEC = 1.0   # 한 스펙트럼을 만들 때 쓸 시간창 길이
NPERSEG = 100      # welch 세그먼트 길이 (FS*WINDOW_SEC 이하로)


def csv_to_windows(csv_path, axis='ax', window_sec=WINDOW_SEC, fs=FS):
    df = pd.read_csv(csv_path)
    window_len = int(window_sec * fs)

    X, y = [], []
    # trial_id별로 분리해서, 각 trial 내에서 슬라이딩 없이 비겹침 윈도우로 자름
    for trial_id, g in df.groupby('trial_id'):
        sig = g[axis].values
        volume = g['volume_pct'].iloc[0]

        n_windows = len(sig) // window_len
        for i in range(n_windows):
            seg = sig[i*window_len:(i+1)*window_len]
            freqs, psd = welch(seg, fs=fs, nperseg=min(NPERSEG, len(seg)))
            X.append(psd)
            y.append(volume)

    return np.array(X), np.array(y)


def main(args):
    csv_files = glob.glob(args.csv_glob)
    if not csv_files:
        raise FileNotFoundError(f"'{args.csv_glob}' 패턴에 맞는 CSV 파일이 없습니다.")

    print(f"발견된 CSV 파일 {len(csv_files)}개: {csv_files}")

    all_X, all_y = [], []
    for path in csv_files:
        X, y = csv_to_windows(path, axis=args.axis)
        all_X.append(X)
        all_y.append(y)
        print(f"  {path}: {len(y)}개 윈도우 생성")

    X = np.concatenate(all_X, axis=0)
    y = np.concatenate(all_y, axis=0)

    print(f"\n최종 데이터: X={X.shape}, y={y.shape}")
    print(f"용량 분포: {sorted(set(y.tolist()))}")

    np.save(args.out_x, X)
    np.save(args.out_y, y)
    print(f"저장 완료: {args.out_x}, {args.out_y}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv_glob", default="log_*.csv", help="CSV 파일 패턴, 예: log_*.csv")
    parser.add_argument("--axis", default="ax", help="사용할 IMU 축 (ax/ay/az)")
    parser.add_argument("--out_x", default="spectra.npy")
    parser.add_argument("--out_y", default="volumes.npy")
    main(parser.parse_args())
