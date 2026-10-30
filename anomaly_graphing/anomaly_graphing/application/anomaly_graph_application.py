import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.ui.window.anomaly_graph_window import AnomalyGraphWindow


class AnomalyGraphApplication:
    def run(self) -> int:
        application = QApplication(sys.argv)
        settings = AnomalyGraphSettings()
        window = AnomalyGraphWindow(settings)
        window.show()
        QTimer.singleShot(0, window.start_loading)
        return application.exec()
