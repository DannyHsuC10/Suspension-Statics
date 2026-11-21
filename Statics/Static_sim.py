import argparse
import sys
from pathlib import Path

from Model_Analysis.simulator_static import run_static


def main():
    default_config = Path(__file__).resolve().parent / "data" / "test_set.json"
    parser = argparse.ArgumentParser(description="執行桿件靜力平衡與材料分析")
    parser.add_argument("config", nargs="?", type=Path, default=default_config, help="載荷設定 JSON")
    args = parser.parse_args()
    run_static(args.config)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    main()
