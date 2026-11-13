"""UDP replay check of a running MOSAIC pipeline with StiffnessProjection (standard library only).

  python replay_udp.py <case>_features.csv <case>_expected.csv [--rate 50] [--in-port 50500] [--out-port 50510]

Sends every feature vector as little-endian doubles to 127.0.0.1:<in-port> (as the ESP32 does), receives the block
output that the pipeline's UDPClient sends to <out-port>, and compares it value by value with the Python reference
(make_reference.py). Stiffness entries: relative error, sigmas and flag: absolute error.
"""
import argparse
import csv
import socket
import struct
import time


def read(path):
    with open(path) as f:
        return [[float(x) for x in row] for row in csv.reader(f) if row]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("features")
    ap.add_argument("expected")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--in-port", type=int, default=50500)
    ap.add_argument("--out-port", type=int, default=50510)
    args = ap.parse_args()
    feats, expected = read(args.features), read(args.expected)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", args.out_port))
    rx.settimeout(2.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    got, worst, worst_i, missing = [], 0.0, -1, 0
    for i, f in enumerate(feats):
        tx.sendto(struct.pack(f"<{len(f)}d", *f), ("127.0.0.1", args.in_port))
        try:
            data, _ = rx.recvfrom(4096)
        except socket.timeout:
            missing += 1
            continue
        out = struct.unpack(f"<{len(data) // 8}d", data)
        got.append(out)
        for k, (a, b) in enumerate(zip(out, expected[i])):
            err = abs(a - b) / max(abs(b), 1.0) if k < 7 else abs(a - b)
            if err > worst:
                worst, worst_i = err, i
        time.sleep(1.0 / args.rate)
    ok = missing == 0 and worst <= 1e-9 and all(len(o) == len(expected[0]) for o in got)
    print(f"{len(feats)} sent, {len(got)} received, {missing} missing, largest error {worst:.2e} (packet {worst_i}) "
          f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
