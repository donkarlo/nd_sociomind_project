from pathlib import Path
import sys

PROJECT_DIRECTORY = Path(__file__).resolve().parent
if str(PROJECT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIRECTORY))

from anomaly_graphing.application.anomaly_graph_application import AnomalyGraphApplication


def main() -> int:
    return AnomalyGraphApplication().run()


if __name__ == "__main__":
    raise SystemExit(main())
