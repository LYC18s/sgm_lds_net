"""Run the stereo perception driver on an AX7020 PYNQ board."""
import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sgm_yolo'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bitfile', type=Path, help='Matching bit/HWH pair; defaults to src/sgm_yolo/overlay/sgm_yolo.bit')
    args = parser.parse_args()
    from dual_ov5640 import SGMYOLOOverlay
    camera = SGMYOLOOverlay(bitfile=args.bitfile, enable_lds_net=True)
    try:
        camera.start()
        while True:
            print(camera.detections(), flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()


if __name__ == '__main__':
    main()
