import argparse
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow import keras
from tensorflow.keras import layers


def build_model(input_len):
    model = keras.Sequential([
        layers.Input(shape=(input_len, 1)),
        layers.Conv1D(8, kernel_size=5, activation='relu'),
        layers.MaxPooling1D(2),
        layers.Conv1D(16, kernel_size=3, activation='relu'),
        layers.GlobalAveragePooling1D(),
        layers.Dense(16, activation='relu'),
        layers.Dense(1)  # 회귀: 용량(%)을 직접 출력
    ])
    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    return model


def main(args):
    X = np.load(args.data_x)
    y_raw = np.load(args.data_y).astype(np.float32)

    # 정규화 (스펙트럼 값 스케일링)
    X = (X - X.mean()) / (X.std() + 1e-8)
    X = X[..., np.newaxis]  # (N, freq_bins, 1)

    # 라벨도 0~1로 정규화 (회귀 타겟 스케일이 크면 MSE 학습이 느려짐)
    y_min, y_max = y_raw.min(), y_raw.max()
    y = (y_raw - y_min) / (y_max - y_min)

    X_train, X_val, y_train, y_val = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    print(f"학습 데이터: {X_train.shape}, 검증 데이터: {X_val.shape}")

    model = build_model(X.shape[1])
    model.summary()

    history = model.fit(
        X_train, y_train,
        validation_data=(X_val, y_val),
        epochs=args.epochs,
        batch_size=args.batch_size,
        verbose=2,
        callbacks=[keras.callbacks.EarlyStopping(patience=15, restore_best_weights=True)]
    )

    val_loss, val_mae = model.evaluate(X_val, y_val, verbose=0)
    real_mae = val_mae * (y_max - y_min)  # 정규화 해제해서 실제 %포인트 오차로 환산
    print(f"\n최종 검증 결과 -> MSE(정규화): {val_loss:.4f}, MAE(정규화): {val_mae:.4f}")
    print(f"실제 단위 환산 MAE: {real_mae:.2f} %포인트")

    model.save(args.out)
    print(f"모델 저장 완료: {args.out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_x", default="spectra.npy")
    parser.add_argument("--data_y", default="volumes.npy")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch_size", type=int, default=16)
    parser.add_argument("--out", default="model.keras")
    main(parser.parse_args())
